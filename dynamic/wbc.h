#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>

#include "mj_adapter.h"
#include "hqp_solver.h"

namespace dynamic {

struct WBCConfig {
    // End-Effector Task gains
    double kp_ee_pos = 150.0;
    double kd_ee_pos = 25.0;
    double ki_ee_pos = 30.0;
    double kp_ee_rot = 100.0;
    double kd_ee_rot = 20.0;

    // Posture Task gains
    double kp_posture = 30.0;
    double kd_posture = 8.0;

    // Base Task gains
    double kp_base = 5000.0;
    double kd_base = 50.0;
    double ki_base = 500.0;

    // Joint velocity damping
    double kd_arm = 2.0;
    double kd_wheel = 1.0;

    // Physical limits
    double qdd_max = 40.0;        // Max joint acceleration (rad/s^2)
    double tau_arm_max = 150.0;   // Max arm torque (Nm)
    double tau_wheel_max = 35.0;  // Max wheel torque (Nm)

    // Flat ground constraint (enforce chassis z, roll, pitch accelerations = 0)
    bool enforce_ground_plane = true;

    // Base mobile mode: if true, base and arm coordinate; if false, base is held stationary
    bool enable_mobile_base = true;
};

struct WBCTarget {
    // End-Effector target in world frame
    Eigen::Vector3d ee_pos_des = Eigen::Vector3d(0.6, 0.0, 0.5);
    Eigen::Matrix3d ee_rot_des = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ee_vel_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d ee_omega_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d ee_acc_des = Eigen::Vector3d::Zero();

    // Nominal arm posture (6 DOFs)
    Eigen::Matrix<double, 6, 1> arm_q_nom = (Eigen::Matrix<double, 6, 1>() << 0.0, -1.0, 1.5, -0.5, 0.0, 0.0).finished();

    // Base target (x, y, yaw)
    Eigen::Vector2d base_pos_des = Eigen::Vector2d::Zero();
    double base_yaw_des = 0.0;

    bool use_orientation = true;
};

class MobileManipulatorWBC {
public:
    MobileManipulatorWBC();
    ~MobileManipulatorWBC();

    // Initialize robot model from URDF
    bool init(const std::string &urdf_path, const WBCConfig &config = WBCConfig());

    // Update internal robot state from MuJoCo Observation
    void update(const Observation &obs);

    // Target setters
    void setTarget(const WBCTarget &target) { target_ = target; }
    void setEETarget(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot = Eigen::Matrix3d::Identity(),
                     const Eigen::Vector3d &vel = Eigen::Vector3d::Zero());
    void setArmNominalPosture(const Eigen::Matrix<double, 6, 1> &q_nom);
    void setBaseTarget(const Eigen::Vector2d &pos, double yaw);

    // Solve WBC HQP and compute MuJoCo actuator Command
    Command solve(double dt);

    // Getters for state and debugging
    Eigen::Vector3d getEEPosition() const;
    Eigen::Matrix3d getEERotation() const;
    Eigen::Vector3d getEEVelocity() const;
    Eigen::Vector3d getEEAngularVelocity() const;
    
    Eigen::Vector3d getBasePosition() const;
    Eigen::Matrix3d getBaseRotation() const;
    double getBaseYaw() const;
    
    Eigen::Matrix<double, 6, 1> getArmQ() const;
    Eigen::Matrix<double, 6, 1> getArmQDot() const;

    const RigidBodyDynamics::Model& getModel() const { return *model_; }
    WBCConfig& config() { return config_; }
    const WBCConfig& config() const { return config_; }
    const WBCTarget& target() const { return target_; }

private:
    std::unique_ptr<RigidBodyDynamics::Model> model_;
    std::unique_ptr<HQPSolver> hqp_solver_;

    WBCConfig config_;
    WBCTarget target_;

    // RBDL state vectors
    RigidBodyDynamics::Math::VectorNd q_;
    RigidBodyDynamics::Math::VectorNd qdot_;
    RigidBodyDynamics::Math::VectorNd qddot_;
    RigidBodyDynamics::Math::MatrixNd M_;
    RigidBodyDynamics::Math::VectorNd h_;

    // Body IDs
    unsigned int base_body_id_ = 2; // chassis_link
    unsigned int ee_body_id_ = 12;  // ur_arm_wrist_3_link
    RigidBodyDynamics::Math::Vector3d ee_point_offset_ = RigidBodyDynamics::Math::Vector3d::Zero();

    // Mechanical parameters for Ridgeback
    double wheel_radius_ = 0.0759;
    double track_half_width_ = 0.2755; // dy
    Eigen::Vector3d pos_err_integral_ = Eigen::Vector3d::Zero();
    double yaw_err_integral_ = 0.0;

    bool initialized_ = false;
};

} // namespace dynamic
