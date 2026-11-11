#include "wbc.h"
#include <iostream>
#include <algorithm>
#include <cmath>

#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace RBD = RigidBodyDynamics;

namespace dynamic
{

    MobileManipulatorWBC::MobileManipulatorWBC()
    {
        model_ = std::make_unique<RBD::Model>();
    }

    MobileManipulatorWBC::~MobileManipulatorWBC() = default;

    bool MobileManipulatorWBC::init(const std::string &urdf_path, const WBCConfig &config)
    {
        config_ = config;
        model_ = std::make_unique<RBD::Model>();

        if (!RBD::Addons::URDFReadFromFile(urdf_path.c_str(), model_.get(), true))
        {
            std::cerr << "[MobileManipulatorWBC] Error reading URDF file: " << urdf_path << std::endl;
            return false;
        }
        std::cout << model_->mJoints[1].mJointType << std::endl;
        if (model_->dof_count != 16 || model_->q_size != 17 || model_->qdot_size != 16)
        {
            std::cerr << "[MobileManipulatorWBC] Unexpected model dimensions! dof: " << model_->dof_count
                      << ", q_size: " << model_->q_size << ", qdot_size: " << model_->qdot_size << std::endl;
            return false;
        }

        base_body_id_ = model_->GetBodyId("chassis_link");
        ee_body_id_ = model_->GetBodyId("ur_arm_wrist_3_link");

        if (base_body_id_ == std::numeric_limits<unsigned int>::max() ||
            ee_body_id_ == std::numeric_limits<unsigned int>::max())
        {
            std::cerr << "[MobileManipulatorWBC] Failed to locate required body IDs: chassis_link or ur_arm_wrist_3_link" << std::endl;
            return false;
        }

        q_ = RBD::Math::VectorNd::Zero(model_->q_size);
        qdot_ = RBD::Math::VectorNd::Zero(model_->qdot_size);
        qddot_ = RBD::Math::VectorNd::Zero(model_->qdot_size);
        M_ = RBD::Math::MatrixNd::Zero(model_->qdot_size, model_->qdot_size);
        h_ = RBD::Math::VectorNd::Zero(model_->qdot_size);

        // Initial unit quaternion for base
        RBD::Math::Quaternion init_quat(0, 0, 0, 1);
        model_->SetQuaternion(base_body_id_, init_quat, q_);

        hqp_solver_ = std::make_unique<HQPSolver>(model_->qdot_size);

        initialized_ = true;
        std::cout << "[MobileManipulatorWBC] Initialized successfully with " << model_->dof_count << " DOFs. Base ID: "
                  << base_body_id_ << ", EE ID: " << ee_body_id_ << std::endl;
        return true;
    }

    void MobileManipulatorWBC::update(const Observation &obs)
    {
        if (!initialized_)
            return;
        if (obs.qpos.size() < 17 || obs.qvel.size() < 16)
            return;

        // 1. Position of base
        q_[0] = obs.qpos[0];
        q_[1] = obs.qpos[1];
        q_[2] = obs.qpos[2];

        // 2. Quaternion of base
        // MuJoCo: [qw, qx, qy, qz] -> RBDL Math::Quaternion(x, y, z, w)
        RBD::Math::Quaternion quat(obs.qpos[4], obs.qpos[5], obs.qpos[6], obs.qpos[3]);
        model_->SetQuaternion(base_body_id_, quat, q_);

        // 3. Wheel positions (front_left, front_right, rear_left, rear_right)
        for (int i = 0; i < 4; ++i)
        {
            q_[6 + i] = obs.qpos[7 + i];
        }

        // 4. Arm joint positions (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
        for (int i = 0; i < 6; ++i)
        {
            q_[10 + i] = obs.qpos[11 + i];
        }

        // 5. Linear velocities of base (world frame)
        qdot_[0] = obs.qvel[0];
        qdot_[1] = obs.qvel[1];
        qdot_[2] = obs.qvel[2];

        // 6. Angular velocities of base
        // MuJoCo free joint qvel[3..5] is body local [wx, wy, wz]
        // RBDL JointTypeSpherical expects angular velocity vector [ωx, ωy, ωz]
        qdot_[3] = obs.qvel[3]; // wx
        qdot_[4] = obs.qvel[4]; // wy
        qdot_[5] = obs.qvel[5]; // wz

        // 7. Wheel velocities
        for (int i = 0; i < 4; ++i)
        {
            qdot_[6 + i] = obs.qvel[6 + i];
        }

        // 8. Arm joint velocities
        for (int i = 0; i < 6; ++i)
        {
            qdot_[10 + i] = obs.qvel[10 + i];
        }

        // 9. Update RBDL kinematics & dynamics
        RBD::UpdateKinematicsCustom(*model_, &q_, &qdot_, nullptr);

        M_.setZero();
        RBD::CompositeRigidBodyAlgorithm(*model_, q_, M_, false);

        h_.setZero();
        RBD::NonlinearEffects(*model_, q_, qdot_, h_);
    }

    void MobileManipulatorWBC::setEETarget(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot, const Eigen::Vector3d &vel)
    {
        target_.ee_pos_des = pos;
        target_.ee_rot_des = rot;
        target_.ee_vel_des = vel;
        target_.use_orientation = true;
    }

    void MobileManipulatorWBC::setArmNominalPosture(const Eigen::Matrix<double, 6, 1> &q_nom)
    {
        target_.arm_q_nom = q_nom;
    }

    void MobileManipulatorWBC::setBaseTarget(const Eigen::Vector2d &pos, double yaw)
    {
        target_.base_pos_des = pos;
        target_.base_yaw_des = yaw;
    }

    Eigen::Vector3d MobileManipulatorWBC::getEEPosition() const
    {
        if (!initialized_)
            return Eigen::Vector3d::Zero();
        RBD::Math::Vector3d pos = RBD::CalcBodyToBaseCoordinates(*model_, q_, ee_body_id_, ee_point_offset_, false);
        return Eigen::Vector3d(pos[0], pos[1], pos[2]);
    }

    Eigen::Matrix3d MobileManipulatorWBC::getEERotation() const
    {
        if (!initialized_)
            return Eigen::Matrix3d::Identity();
        // CalcBodyWorldOrientation returns E (world to body), transpose to get body to world
        RBD::Math::Matrix3d E = RBD::CalcBodyWorldOrientation(*model_, q_, ee_body_id_, false);
        Eigen::Matrix3d rot;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                rot(r, c) = E(c, r);
            }
        }
        return rot;
    }

    Eigen::Vector3d MobileManipulatorWBC::getEEVelocity() const
    {
        if (!initialized_)
            return Eigen::Vector3d::Zero();
        RBD::Math::SpatialVector vel6D = RBD::CalcPointVelocity6D(*model_, q_, qdot_, ee_body_id_, ee_point_offset_, false);
        // Linear velocity is last 3 elements
        return Eigen::Vector3d(vel6D[3], vel6D[4], vel6D[5]);
    }

    Eigen::Vector3d MobileManipulatorWBC::getEEAngularVelocity() const
    {
        if (!initialized_)
            return Eigen::Vector3d::Zero();
        RBD::Math::SpatialVector vel6D = RBD::CalcPointVelocity6D(*model_, q_, qdot_, ee_body_id_, ee_point_offset_, false);
        // Angular velocity is first 3 elements
        return Eigen::Vector3d(vel6D[0], vel6D[1], vel6D[2]);
    }

    Eigen::Vector3d MobileManipulatorWBC::getBasePosition() const
    {
        if (!initialized_)
            return Eigen::Vector3d::Zero();
        RBD::Math::Vector3d pos = RBD::CalcBodyToBaseCoordinates(*model_, q_, base_body_id_, RBD::Math::Vector3d::Zero(), false);
        return Eigen::Vector3d(pos[0], pos[1], pos[2]);
    }

    Eigen::Matrix3d MobileManipulatorWBC::getBaseRotation() const
    {
        if (!initialized_)
            return Eigen::Matrix3d::Identity();
        RBD::Math::Matrix3d E = RBD::CalcBodyWorldOrientation(*model_, q_, base_body_id_, false);
        Eigen::Matrix3d rot;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                rot(r, c) = E(c, r);
            }
        }
        return rot;
    }

    double MobileManipulatorWBC::getBaseYaw() const
    {
        Eigen::Matrix3d rot = getBaseRotation();
        return std::atan2(rot(1, 0), rot(0, 0));
    }

    Eigen::Matrix<double, 6, 1> MobileManipulatorWBC::getArmQ() const
    {
        Eigen::Matrix<double, 6, 1> arm_q;
        for (int i = 0; i < 6; ++i)
        {
            arm_q[i] = q_[10 + i];
        }
        return arm_q;
    }

    Eigen::Matrix<double, 6, 1> MobileManipulatorWBC::getArmQDot() const
    {
        Eigen::Matrix<double, 6, 1> arm_qdot;
        for (int i = 0; i < 6; ++i)
        {
            arm_qdot[i] = qdot_[10 + i];
        }
        return arm_qdot;
    }

    Command MobileManipulatorWBC::solve(double dt)
    {
        Command cmd;
        cmd.ctrl.assign(10, 0.0);
        if (!initialized_)
            return cmd;

        hqp_solver_->reset();

        // 1. Variable acceleration bounds
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(16, -config_.qdd_max);
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(16, config_.qdd_max);

        // Ground plane constraints (keep chassis flat on ground)
        if (config_.enforce_ground_plane)
        {
            lb[2] = -1e-3;
            ub[2] = 1e-3; // no vertical acceleration TZ
            lb[4] = -1e-3;
            ub[4] = 1e-3; // no pitch acceleration RY
            lb[3] = -1e-3;
            ub[3] = 1e-3; // no roll acceleration RX
        }

        // If mobile base coordination is disabled, freeze base planar DOFs and wheels
        if (!config_.enable_mobile_base)
        {
            lb[0] = -1e-3;
            ub[0] = 1e-3; // TX
            lb[1] = -1e-3;
            ub[1] = 1e-3; // TY
            lb[5] = -1e-3;
            ub[5] = 1e-3; // RZ
            for (int i = 6; i < 10; ++i)
            {
                lb[i] = -1e-3;
                ub[i] = 1e-3;
            }
        }

        // Arm velocity limits barrier
        const double arm_vel_max = 3.14;
        for (int i = 0; i < 6; ++i)
        {
            int idx = 10 + i;
            double a_max_vel = (arm_vel_max - qdot_[idx]) / dt;
            double a_min_vel = (-arm_vel_max - qdot_[idx]) / dt;
            lb[idx] = std::max(lb[idx], a_min_vel);
            ub[idx] = std::min(ub[idx], a_max_vel);
            if (lb[idx] > ub[idx])
            {
                double mid = 0.5 * (lb[idx] + ub[idx]);
                lb[idx] = mid - 1.0;
                ub[idx] = mid + 1.0;
            }
        }

        hqp_solver_->setVariableBounds(lb, ub);

        // 2. Task Level 0: End-Effector Tracking
        Eigen::Vector3d p_curr = getEEPosition();
        Eigen::Matrix3d R_curr = getEERotation();
        Eigen::Vector3d v_curr = getEEVelocity();
        Eigen::Vector3d w_curr = getEEAngularVelocity();

        // Position desired acceleration
        Eigen::Vector3d pos_err = target_.ee_pos_des - p_curr;
        Eigen::Vector3d vel_err = target_.ee_vel_des - v_curr;
        pos_err_integral_ += pos_err * dt;
        for (int i = 0; i < 3; ++i)
        {
            pos_err_integral_[i] = std::clamp(pos_err_integral_[i], -0.2, 0.2);
        }
        Eigen::Vector3d a_pos_des = target_.ee_acc_des + config_.kp_ee_pos * pos_err + config_.kd_ee_pos * vel_err + config_.ki_ee_pos * pos_err_integral_;

        // Orientation desired acceleration via SO(3) skew-symmetric error
        Eigen::Matrix3d R_err_mat = target_.ee_rot_des * R_curr.transpose();
        Eigen::Vector3d rot_err = 0.5 * Eigen::Vector3d(
                                            R_err_mat(2, 1) - R_err_mat(1, 2),
                                            R_err_mat(0, 2) - R_err_mat(2, 0),
                                            R_err_mat(1, 0) - R_err_mat(0, 1));
        Eigen::Vector3d a_rot_des = config_.kp_ee_rot * rot_err + config_.kd_ee_rot * (target_.ee_omega_des - w_curr);

        // Compute EE 6D Jacobian and bias acceleration dJ*qdot
        RBD::Math::MatrixNd J6D_rbd = RBD::Math::MatrixNd::Zero(6, 16);
        RBD::CalcPointJacobian6D(*model_, q_, ee_body_id_, ee_point_offset_, J6D_rbd, false);

        RBD::Math::VectorNd qddot_zero = RBD::Math::VectorNd::Zero(16);
        RBD::Math::SpatialVector bias6D = RBD::CalcPointAcceleration6D(*model_, q_, qdot_, qddot_zero, ee_body_id_, ee_point_offset_, true);

        if (target_.use_orientation)
        {
            Eigen::MatrixXd A_ee = J6D_rbd;
            Eigen::VectorXd b_ee(6);
            // RBDL J6D: rows 0..2 angular, rows 3..5 linear
            b_ee.head<3>() = a_rot_des - Eigen::Vector3d(bias6D[0], bias6D[1], bias6D[2]);
            b_ee.tail<3>() = a_pos_des - Eigen::Vector3d(bias6D[3], bias6D[4], bias6D[5]);
            hqp_solver_->addTask(0, HQPTask{"EE_Pose_Tracking", A_ee, b_ee, 1.0});
        }
        else
        {
            Eigen::MatrixXd A_ee = J6D_rbd.bottomRows(3);
            Eigen::VectorXd b_ee = a_pos_des - Eigen::Vector3d(bias6D[3], bias6D[4], bias6D[5]);
            hqp_solver_->addTask(0, HQPTask{"EE_Position_Tracking", A_ee, b_ee, 1.0});
        }

        Eigen::VectorXd b_base = Eigen::VectorXd::Zero(1);
        if (config_.enable_mobile_base)
        {
            // Base planar posture task (x, y, yaw)
            Eigen::MatrixXd A_base = Eigen::MatrixXd::Zero(1, 16);
            A_base(0, 5) = 1.0; // RZ
       

            double yaw_curr = getBaseYaw();
            double yaw_diff = target_.base_yaw_des - yaw_curr;
            while (yaw_diff > M_PI)
                yaw_diff -= 2.0 * M_PI;
            while (yaw_diff < -M_PI)
                yaw_diff += 2.0 * M_PI;


            yaw_err_integral_ += yaw_diff * dt;
            yaw_err_integral_ = std::clamp(yaw_err_integral_, -10.0,10.0);
            b_base[0] = config_.kp_base * yaw_diff  + config_.ki_base * yaw_err_integral_;

            hqp_solver_->addTask(1, HQPTask{"Base_Posture", A_base, b_base, 1.0});
        }

        // 4. Task Level 2: Minimal Acceleration Regularization
        Eigen::MatrixXd A_reg = Eigen::MatrixXd::Identity(16, 16);
        Eigen::VectorXd b_reg = Eigen::VectorXd::Zero(16);
        hqp_solver_->addTask(2, HQPTask{"Regularization", A_reg, b_reg, 0.01});

        // Solve HQP
        Eigen::VectorXd qddot_opt;
        bool success = hqp_solver_->solve(qddot_opt);
        if (!success)
        {
            qddot_opt = Eigen::VectorXd::Zero(16);
        }

        Eigen::VectorXd tau_gen = M_ * qddot_opt + h_;

    
        for (int i = 0; i < 6; ++i)
        {
            double tau_arm = tau_gen[10 + i] - config_.kd_arm * qdot_[10 + i];
            tau_arm = std::clamp(tau_arm, -config_.tau_arm_max, config_.tau_arm_max);
            cmd.ctrl[4 + i] = tau_arm;
        }

    
        if (config_.enable_mobile_base)
        {
            double yaw = getBaseYaw();
            double Fx = tau_gen[0] * std::cos(yaw) + tau_gen[1] * std::sin(yaw);
            double Mz = tau_gen[5];

            double tau_fwd = (Fx / 4.0) * wheel_radius_;
            double tau_yaw = (Mz / (4.0 * track_half_width_)) * wheel_radius_;

            double fl = tau_fwd - tau_yaw;
            double fr = tau_fwd + tau_yaw;
            double rl = tau_fwd - tau_yaw;
            double rr = tau_fwd + tau_yaw;

            cmd.ctrl[0] = std::clamp(fl, -config_.tau_wheel_max, config_.tau_wheel_max);
            cmd.ctrl[1] = std::clamp(fr, -config_.tau_wheel_max, config_.tau_wheel_max);
            cmd.ctrl[2] = std::clamp(rl, -config_.tau_wheel_max, config_.tau_wheel_max);
            cmd.ctrl[3] = std::clamp(rr, -config_.tau_wheel_max, config_.tau_wheel_max);
        }
        else
        {
           
            for (int i = 0; i < 4; ++i)
            {
                double tau_w = -config_.kd_wheel * 5.0 * qdot_[6 + i];
                cmd.ctrl[i] = std::clamp(tau_w, -config_.tau_wheel_max, config_.tau_wheel_max);
            }
        }

        return cmd;
    }

} 
