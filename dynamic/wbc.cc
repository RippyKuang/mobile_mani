#include "wbc.h"
#include <iostream>
#include <algorithm>
#include <cmath>

#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace RBD = RigidBodyDynamics;

namespace dynamic
{

    inline double wrapToPi(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    inline double Integrate(double error, double integral, double dt)
    {

        return integral + error * dt;
    }

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

        base_body_id_ = model_->GetBodyId("chassis_link");
        ee_body_id_ = model_->GetBodyId("ur_arm_wrist_3_link");

        static const char *arm_link_names[6] = {
            "ur_arm_shoulder_link", "ur_arm_upper_arm_link", "ur_arm_forearm_link",
            "ur_arm_wrist_1_link", "ur_arm_wrist_2_link", "ur_arm_wrist_3_link"};
        for (int i = 0; i < 6; ++i)
        {
            const unsigned int body_id = model_->GetBodyId(arm_link_names[i]);

            const RBD::Joint &joint = model_->mJoints[body_id];

            arm_q_idx_[i] = static_cast<int>(joint.q_index);
        }

        nv_ = model_->qdot_size;

        q_ = RBD::Math::VectorNd::Zero(model_->q_size);
        qdot_ = RBD::Math::VectorNd::Zero(nv_);
        qddot_ = RBD::Math::VectorNd::Zero(nv_);
        M_ = RBD::Math::MatrixNd::Zero(nv_, nv_);
        h_ = RBD::Math::VectorNd::Zero(nv_);

        RBD::Math::Quaternion init_quat(0, 0, 0, 1);
        model_->SetQuaternion(base_body_id_, init_quat, q_);

        hqp_solver_ = std::make_unique<HQPSolver>(nv_);

        std::cout << "[MobileManipulatorWBC] Initialized successfully with " << model_->dof_count << " DOFs. Base ID: "
                  << base_body_id_ << ", EE ID: " << ee_body_id_ << std::endl;
        return true;
    }

    void MobileManipulatorWBC::update(const Observation &obs)
    {
        q_[0] = obs.qpos[0];
        q_[1] = obs.qpos[1];
        q_[2] = obs.qpos[2];

        RBD::Math::Quaternion quat(obs.qpos[4], obs.qpos[5], obs.qpos[6], obs.qpos[3]);
        model_->SetQuaternion(base_body_id_, quat, q_);

        for (int i = 0; i < 6; ++i)
            q_[arm_q_idx_[i]] = obs.qpos[11 + i];

        qdot_[0] = obs.qvel[0];
        qdot_[1] = obs.qvel[1];
        qdot_[2] = obs.qvel[2];

        qdot_[3] = obs.qvel[3]; // wx
        qdot_[4] = obs.qvel[4]; // wy
        qdot_[5] = obs.qvel[5]; // wz

        for (int i = 0; i < 6; ++i)
            qdot_[arm_q_idx_[i]] = obs.qvel[10 + i];

        // 8. Update RBDL kinematics & dynamics
        RBD::UpdateKinematicsCustom(*model_, &q_, &qdot_, nullptr);

        M_.setZero();
        RBD::CompositeRigidBodyAlgorithm(*model_, q_, M_, false);

        h_.setZero();
        RBD::NonlinearEffects(*model_, q_, qdot_, h_);
    }

    void MobileManipulatorWBC::setEETarget(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
                                           const Eigen::Vector3d &vel, bool use_orientation)
    {
        target_.ee_pos_des = pos;
        target_.ee_rot_des = rot;
        target_.ee_vel_des = vel;
        target_.use_orientation = use_orientation;
    }

    void MobileManipulatorWBC::setArmNominalPosture(const Eigen::Matrix<double, 6, 1> &q_nom)
    {
        target_.arm_q_nom = q_nom;
    }

    void MobileManipulatorWBC::setBaseTarget(const Eigen::Vector2d &pos, double yaw,
                                             double yaw_rate, double forward_vel)
    {
        target_.base_pos_des = pos;
        target_.base_yaw_des = yaw;
        target_.base_yaw_rate_des = yaw_rate;
        target_.base_forward_vel_des = forward_vel;
    }

    Eigen::Vector3d MobileManipulatorWBC::getEEPosition() const
    {

        RBD::Math::Vector3d pos = RBD::CalcBodyToBaseCoordinates(*model_, q_, ee_body_id_, ee_point_offset_, false);
        return Eigen::Vector3d(pos[0], pos[1], pos[2]);
    }

    Eigen::Matrix3d MobileManipulatorWBC::getEERotation() const
    {

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

        RBD::Math::SpatialVector vel6D = RBD::CalcPointVelocity6D(*model_, q_, qdot_, ee_body_id_, ee_point_offset_, false);
        // Linear velocity is last 3 elements
        return Eigen::Vector3d(vel6D[3], vel6D[4], vel6D[5]);
    }

    Eigen::Vector3d MobileManipulatorWBC::getEEAngularVelocity() const
    {

        RBD::Math::SpatialVector vel6D = RBD::CalcPointVelocity6D(*model_, q_, qdot_, ee_body_id_, ee_point_offset_, false);
        // Angular velocity is first 3 elements
        return Eigen::Vector3d(vel6D[0], vel6D[1], vel6D[2]);
    }

    Eigen::Vector3d MobileManipulatorWBC::getBasePosition() const
    {

        RBD::Math::Vector3d pos = RBD::CalcBodyToBaseCoordinates(*model_, q_, base_body_id_, RBD::Math::Vector3d::Zero(), false);
        return Eigen::Vector3d(pos[0], pos[1], pos[2]);
    }

    Eigen::Matrix3d MobileManipulatorWBC::getBaseRotation() const
    {

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

    double MobileManipulatorWBC::getBaseAngularVelocity() const
    {
        return qdot_[5];
    }

    Eigen::Matrix<double, 6, 1> MobileManipulatorWBC::getArmQ() const
    {
        Eigen::Matrix<double, 6, 1> arm_q;
        for (int i = 0; i < 6; ++i)
            arm_q[i] = q_[arm_q_idx_[i]];
        return arm_q;
    }

    Eigen::Matrix<double, 6, 1> MobileManipulatorWBC::getArmQDot() const
    {
        Eigen::Matrix<double, 6, 1> arm_qdot;
        for (int i = 0; i < 6; ++i)
            arm_qdot[i] = qdot_[arm_q_idx_[i]];
        return arm_qdot;
    }

    Command MobileManipulatorWBC::solve(double dt)
    {
        Command cmd;
        cmd.ctrl.assign(10, 0.0);

        hqp_solver_->reset();
        const double yaw = getBaseYaw();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        // 0. 非完整约束：差速底盘不能产生横向加速度。
        //    横向方向 = 航向的法向 (-sin, cos)。不加这条，EE 任务会通过
        //    "横向推底盘"来跟踪末端（轮子做不到），导致打滑、漂移、末端跟不上。

        Eigen::MatrixXd C_nh = Eigen::MatrixXd::Zero(1, nv_);
        C_nh(0, 0) = -s;
        C_nh(0, 1) = c;
        hqp_solver_->setLinearConstraints(
            C_nh,
            Eigen::VectorXd::Constant(1, -config_.base_lateral_acc_max),
            Eigen::VectorXd::Constant(1, config_.base_lateral_acc_max));

        // 1. Variable acceleration bounds
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(nv_, -config_.qdd_max);
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(nv_, config_.qdd_max);

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

        // Arm velocity limits barrier
        const double arm_vel_max = 3.14;
        if (dt > 0.0)
        {
            for (int i = 0; i < 6; ++i)
            {
                int idx = arm_q_idx_[i];
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
        }

        hqp_solver_->setVariableBounds(lb, ub);

        // Task Level 1: End-Effector Tracking
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
        RBD::Math::MatrixNd J6D_rbd = RBD::Math::MatrixNd::Zero(6, nv_);
        RBD::CalcPointJacobian6D(*model_, q_, ee_body_id_, ee_point_offset_, J6D_rbd, false);

        RBD::Math::VectorNd qddot_zero = RBD::Math::VectorNd::Zero(nv_);
        RBD::Math::SpatialVector bias6D = RBD::CalcPointAcceleration6D(*model_, q_, qdot_, qddot_zero, ee_body_id_, ee_point_offset_, true);

        if (target_.use_orientation)
        {
            Eigen::MatrixXd A_ee = J6D_rbd;
            Eigen::VectorXd b_ee(6);

            b_ee.head<3>() = a_rot_des - Eigen::Vector3d(bias6D[0], bias6D[1], bias6D[2]);
            b_ee.tail<3>() = a_pos_des - Eigen::Vector3d(bias6D[3], bias6D[4], bias6D[5]);
            hqp_solver_->addTask(1, HQPTask{"EE_Pose_Tracking", A_ee, b_ee, 1.0, 0.5});
        }
        else
        {
            Eigen::MatrixXd A_ee = J6D_rbd.bottomRows(3);
            Eigen::VectorXd b_ee = a_pos_des - Eigen::Vector3d(bias6D[3], bias6D[4], bias6D[5]);
            hqp_solver_->addTask(1, HQPTask{"EE_Position_Tracking", A_ee, b_ee, 1.0, 0.5});
        }

        // Task Level 0: base planar tracking

        Eigen::MatrixXd A_base = Eigen::MatrixXd::Zero(2, nv_);

        A_base(0, 0) = c; // body-forward direction in the world frame
        A_base(0, 1) = s;
        A_base(1, 5) = 1.0; // yaw acceleration

        // ---- yaw loop ----
        const double yaw_err = std::abs(wrapToPi(target_.base_yaw_des - yaw));
        const double yaw_rate = qdot_[5];
        yaw_err_integral_ = Integrate(yaw_err, yaw_err_integral_, dt);
        const double yaw_int = config_.ki_base_yaw * std::clamp(yaw_err_integral_,
                                                                -config_.base_int_limit, config_.base_int_limit);
        double a_yaw_des = config_.kp_base_yaw * yaw_err +
                           config_.kd_base_yaw * (target_.base_yaw_rate_des - yaw_rate) +
                           yaw_int;

        // ---- forward (body x) loop ----
        const Eigen::Vector3d base_pos = getBasePosition();
        const Eigen::Vector2d err_world =
            target_.base_pos_des - Eigen::Vector2d(base_pos.x(), base_pos.y());
        const double e_fwd = err_world.dot(Eigen::Vector2d(c, s));
        const double v_fwd = qdot_[0] * c + qdot_[1] * s;
        base_err_integral_ = Integrate(e_fwd, base_err_integral_, dt);
        const double fwd_int = config_.ki_base_x * std::clamp(base_err_integral_,
                                                              -config_.base_int_limit, config_.base_int_limit);
        double a_fwd_des = config_.kp_base_x * e_fwd +
                           config_.kd_base_x * (target_.base_forward_vel_des - v_fwd) +
                           fwd_int;

        a_fwd_des = std::clamp(a_fwd_des, -config_.base_acc_limit, config_.base_acc_limit);
        a_yaw_des = std::clamp(a_yaw_des, -config_.base_acc_limit, config_.base_acc_limit);

        Eigen::VectorXd b_base(2);
        b_base[0] = a_fwd_des;
        b_base[1] = a_yaw_des;
        hqp_solver_->addTask(0, HQPTask{"Base_Planar", A_base, b_base, 1.0, 0.1});

        // Task Level 2: arm posture in the remaining null space.

        if (config_.kp_posture > 0.0 && !target_.use_orientation)
        {
            Eigen::MatrixXd A_post = Eigen::MatrixXd::Zero(6, nv_);
            Eigen::VectorXd b_post(6);
            for (int i = 0; i < 6; ++i)
            {
                const int dof = arm_q_idx_[i];
                A_post(i, dof) = 1.0;
                b_post[i] = config_.kp_posture * (target_.arm_q_nom[i] - q_[arm_q_idx_[i]]) -
                            config_.kd_posture * qdot_[dof];
            }
            hqp_solver_->addTask(2, HQPTask{"Arm_Posture", A_post, b_post, 0.1, 0.1});
        }

        // 5. Task Level 3: minimal acceleration regularization
        Eigen::MatrixXd A_reg = Eigen::MatrixXd::Identity(nv_, nv_);
        Eigen::VectorXd b_reg = Eigen::VectorXd::Zero(nv_);
        hqp_solver_->addTask(3, HQPTask{"Regularization", A_reg, b_reg, 0.01});

        // Solve HQP
        Eigen::VectorXd qddot_opt;
        bool success = hqp_solver_->solve(qddot_opt);
        if (!success)
        {
            qddot_opt = Eigen::VectorXd::Zero(nv_);
        }

        Eigen::VectorXd tau_gen = M_ * qddot_opt + h_;

        for (int i = 0; i < 6; ++i)
        {
            const int dof = arm_q_idx_[i];
            double tau_arm = tau_gen[dof] - config_.kd_arm * qdot_[dof];
            tau_arm = std::clamp(tau_arm, -config_.tau_arm_max, config_.tau_arm_max);
            cmd.ctrl[4 + i] = tau_arm;
        }

        // tau_gen[0..5] is the net wrench on the floating base, expressed in the
        // world frame for TX/TY and in the body frame for RZ.

        const double Fx = tau_gen[0] * c + tau_gen[1] * s;
        double ff = 300 + 50 * (target_.base_yaw_rate_des - qdot_[5]) + yaw_err_integral_;
        //     std::cout << target_.base_yaw_rate_des - qdot_[5] << std::endl;

        const double Mz = tau_gen[5] + ff * std::tanh((target_.base_yaw_rate_des - qdot_[5]) / 0.005);

        double tau_fwd = (Fx / 4.0) * wheel_radius_;
        double tau_yaw = (Mz / (4.0 * track_half_width_)) * wheel_radius_;

       
        const double limit = config_.tau_wheel_max;
      
        tau_yaw = std::clamp(tau_yaw, -limit, limit);

        const double room_fwd = std::max(0.0, limit - std::abs(tau_yaw));
        tau_fwd = std::clamp(tau_fwd, -room_fwd, room_fwd);

        const double fl = std::clamp(tau_fwd - tau_yaw, -config_.tau_wheel_max, config_.tau_wheel_max);
        const double fr = std::clamp(tau_fwd + tau_yaw, -config_.tau_wheel_max, config_.tau_wheel_max);
        const double rl = fl;
        const double rr = fr;

        cmd.ctrl[0] = fl;
        cmd.ctrl[1] = fr;
        cmd.ctrl[2] = rl;
        cmd.ctrl[3] = rr;

        return cmd;
    }

}
