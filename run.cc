#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <csignal>
#include <atomic>
#include <iomanip>

#include <filesystem>

#include "mj_adapter.h"
#include "wbc.h"


int main()
{
  

    std::cout << "========================================================\n";
    std::cout << "  Ridgeback-UR5 Mobile Manipulator WBC HQP Controller   \n";
    std::cout << "========================================================\n" << std::endl;

    // Resolve model paths
    std::string scene_xml = "../ridgeback_ur5/mjcf/scene.xml";
    std::string robot_urdf = "../ridgeback_ur5/urdf/ridgeback_ur5.urdf";
    if (!std::filesystem::exists(scene_xml)) {
        scene_xml = "ridgeback_ur5/mjcf/scene.xml";
        robot_urdf = "ridgeback_ur5/urdf/ridgeback_ur5.urdf";
    }

    // 1. Start MuJoCo simulation environment
    std::cout << "[run] Initializing MuJoCo simulation with: " << scene_xml << std::endl;
    MjAdapter mj(scene_xml.c_str());

    // 2. Initialize WBC HQP controller
    std::cout << "[run] Initializing Mobile Manipulator WBC controller with: " << robot_urdf << std::endl;
    dynamic::MobileManipulatorWBC wbc;
    dynamic::WBCConfig config;
    config.kp_ee_pos = 120.0;
    config.kd_ee_pos = 22.0;
    config.kp_ee_rot = 80.0;
    config.kd_ee_rot = 18.0;
    config.kp_posture = 25.0;
    config.kd_posture = 8.0;
    
    config.enable_mobile_base = true;

    if (!wbc.init(robot_urdf, config)) {
        std::cerr << "[run] Failed to initialize WBC controller!" << std::endl;
        return 1;
    }

    // 3. Wait for initial observation from MuJoCo
    std::cout << "[run] Waiting for initial MuJoCo observation..." << std::endl;
    Observation obs;
    while (obs.qpos.empty()) {
        obs = mj.get_obs();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  

    wbc.update(obs);

    // Initial EE position & orientation
    Eigen::Vector3d init_ee_pos = wbc.getEEPosition();
    Eigen::Matrix3d init_ee_rot = wbc.getEERotation();
    Eigen::Vector3d init_base_pos = wbc.getBasePosition();
    double init_base_yaw = wbc.getBaseYaw();

  // Set initial EE X position to 0.6 m in front of the base
    std::cout << "\n[run] Initial State:" << std::endl;
    std::cout << "  Base Position: " << init_base_pos.transpose() << " m, Yaw: " << init_base_yaw << " rad" << std::endl;
    std::cout << "  EE Position:   " << init_ee_pos.transpose() << " m" << std::endl;
    std::cout << "  Arm Q:         " << wbc.getArmQ().transpose() << " rad\n" << std::endl;

    // Set initial targets
    wbc.setArmNominalPosture(wbc.getArmQ());
    wbc.setBaseTarget(Eigen::Vector2d(init_base_pos.x(), init_base_pos.y()), init_base_yaw);

    // 4. Real-time control loop at 500 Hz (dt = 0.002s)
    const double dt = 0.002;
    double sim_time = 0.0;
    auto last_log_time = std::chrono::steady_clock::now();

    std::cout << "[run] Starting 500Hz WBC control loop. Press Ctrl+C or close window to exit.\n" << std::endl;

    while (true) {
        auto loop_start = std::chrono::steady_clock::now();

        // 4.1 Update WBC state from latest MuJoCo observation
        obs = mj.get_obs();
        if (!obs.qpos.empty()) {
            wbc.update(obs);
        }

        // 4.2 Generate smooth 3D Cartesian trajectory for End-Effector
        // Traces a gentle circle in the Y-Z plane
        const double omega = 0.8; // rad/s
        const double radius_y = 0.8; // 8 cm
        const double radius_z = 0.5; // 5 cm

        Eigen::Vector3d target_pos = init_ee_pos;
        Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();

        if (sim_time > 1.0) {
            double t_traj = sim_time - 1.0;
            target_pos.x() += radius_y * std::sin(omega * t_traj);
            target_pos.y() += radius_z * (1.0 - std::cos(omega * t_traj));

            target_vel.x() = radius_y * omega * std::cos(omega * t_traj);
            target_vel.y() = radius_z * omega * std::sin(omega * t_traj);
        }

        wbc.setEETarget(target_pos, init_ee_rot, target_vel);

        // 4.3 Solve HQP & Inverse Dynamics
        Command cmd = wbc.solve(dt);

        // 4.4 Send computed torque command to MuJoCo
        mj.set_cmd(cmd);

        sim_time += dt;

        // 4.5 Periodic terminal logging (every 1.0 s)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_log_time).count() >= 1.0) {
            last_log_time = now;
            Eigen::Vector3d curr_ee = wbc.getEEPosition();
            double pos_err_norm = (target_pos - curr_ee).norm();
            std::cout << std::fixed << std::setprecision(3)
                      << "[t=" << sim_time << "s] "
                      << "Target EE: [" << target_pos.x() << ", " << target_pos.y() << ", " << target_pos.z() << "] | "
                      << "Actual EE: [" << curr_ee.x() << ", " << curr_ee.y() << ", " << curr_ee.z() << "] | "
                      << "Err: " << std::setprecision(4) << pos_err_norm * 1000.0 << " mm"
                      << std::endl;
        }

        // 4.6 Maintain loop timing (500Hz)
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        auto sleep_time = std::chrono::duration<double>(dt) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

   
}