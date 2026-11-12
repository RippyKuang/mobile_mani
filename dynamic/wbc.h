#pragma once

#include <array>
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
    double kp_ee_pos = 300.0;
    double kd_ee_pos = 25.0;
    double ki_ee_pos = 100.0;
    double kp_ee_rot = 100.0;
    double kd_ee_rot = 20.0;

    // Posture Task gains
    double kp_posture = 30.0;
    double kd_posture = 8.0;

    // Base Task gains. The base task lives in the *body* frame: a differential
    // drive can only push forward and turn, so the planar error is projected onto
    // the heading. Both rows are accelerations, hence kp in 1/s^2 and kd in 1/s.
    double kp_base_x = 4.0;       // 1/s^2
    double kd_base_x = 4.0;       // 1/s
    double ki_base_x = 0.0;       // 1/s^3
    double kp_base_yaw = 3.0;     // 1/s^2 (low: the yaw plant is stiction dominated)
    double kd_base_yaw = 5.0;     // 1/s
    double ki_base_yaw = 0.0;     // 1/s^3
    double base_acc_limit = 10.0; // clamp on the base task desired accel (m/s^2, rad/s^2);
                              // the position loop does not need qdd_max and a huge value
                              // only makes it steal the wheel torque budget
    // A differential drive cannot accelerate sideways. If the QP is allowed to use the
    // base's lateral DOF, the end-effector task will "cheat" by asking the base to slide
    // sideways, which the wheels physically cannot do -> wheel slip, base drift and the
    // end effector never catches up. Enforce it as a hard constraint on every level.
    double base_lateral_acc_max = 0.5; // m/s^2, tolerance around zero lateral accel
    double base_int_limit = 25.0; // clamp on each integral contribution (accel units);
                              // must be large enough that ki*integral can reach the skid
                              // breakaway torque (~26 Nm of differential => 24 rad/s^2)

    // Joint velocity damping
    double kd_arm = 2.0;
    double kd_wheel = 1.0;

    // Physical limits
    double qdd_max = 40.0;        // Max joint acceleration (rad/s^2)
    double tau_arm_max = 150.0;   // Max arm torque (Nm)
    double tau_wheel_max = 35.0;  // Max wheel torque (Nm)

  
    // The floor now declares friction explicitly in scene.xml, so the wheels' 0.5 is
    // no longer overridden by MuJoCo's element-wise max() combination rule.
    double wheel_friction = 0.5;  // Effective friction coefficient for the traction cap
    double wheel_load_fraction = 0.25; // Static load share per wheel
    double robot_mass = 220.3;    // kg, used for the per-wheel normal force

    // Flat ground constraint (enforce chassis z, roll, pitch accelerations = 0)
    bool enforce_ground_plane = true;

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

    // Base target (x, y, yaw) plus optional velocity feed-forward
    Eigen::Vector2d base_pos_des = Eigen::Vector2d::Zero();
    double base_yaw_des = 0.0;
    double base_yaw_rate_des = 0.0;    // rad/s, feed-forward (removes ramp lag)
    double base_forward_vel_des = 0.0; // m/s, body frame, feed-forward

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
                     const Eigen::Vector3d &vel = Eigen::Vector3d::Zero(),
                     bool use_orientation = true);
    void setArmNominalPosture(const Eigen::Matrix<double, 6, 1> &q_nom);
  
    void setBaseTarget(const Eigen::Vector2d &pos, double yaw, double yaw_rate = 0.0, double forward_vel = 0.0);

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
    double getBaseAngularVelocity() const;
    
    Eigen::Matrix<double, 6, 1> getArmQ() const;
    Eigen::Matrix<double, 6, 1> getArmQDot() const;


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

    // Model layout: 12 DoF = floating base (6) + arm (6);
    // q layout: [0..2]=base xyz, [3..5]=quat xyz, [6..11]=arm, [12]=quat w (RBDL 3.x).
    int nv_ = 0;
    std::array<int, 6> arm_q_idx_{};


    // Body IDs 
    unsigned int base_body_id_ = 0;
    unsigned int ee_body_id_ = 0;
    RigidBodyDynamics::Math::Vector3d ee_point_offset_ = RigidBodyDynamics::Math::Vector3d::Zero();

    // Mechanical parameters for Ridgeback
    double wheel_radius_ = 0.0759;
    double track_half_width_ = 0.2755; // dy



    Eigen::Vector3d pos_err_integral_ = Eigen::Vector3d::Zero();
    double base_err_integral_ = 0.0; // body-forward position error integral
    double yaw_err_integral_ = 0.0;

};

} 
