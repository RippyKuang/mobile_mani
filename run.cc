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

    std::string scene_xml = "../ridgeback_ur5/mjcf/scene.xml";
    std::string robot_urdf = "../ridgeback_ur5/urdf/ridgeback_ur5.urdf";
    if (!std::filesystem::exists(scene_xml))
    {
        scene_xml = "ridgeback_ur5/mjcf/scene.xml";
        robot_urdf = "ridgeback_ur5/urdf/ridgeback_ur5.urdf";
    }

    MjAdapter mj(scene_xml.c_str());

    dynamic::MobileManipulatorWBC wbc;
    dynamic::WBCConfig config;
    config.kp_ee_pos = 120.0;
    config.kd_ee_pos = 22.0;
    config.kp_ee_rot = 80.0;
    config.kd_ee_rot = 18.0;
    config.kp_posture = 25.0;
    config.kd_posture = 8.0;

    config.enable_mobile_base = true;

    if (!wbc.init(robot_urdf, config))
    {
        std::cerr << "[run] Failed to initialize WBC controller!" << std::endl;
        return 1;
    }

    std::cout << "[run] Waiting for initial MuJoCo observation..." << std::endl;
    Observation obs;
    while (obs.qpos.empty())
    {
        obs = mj.get_obs();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    wbc.update(obs);

    Eigen::Vector3d init_ee_pos = wbc.getEEPosition();
    Eigen::Matrix3d init_ee_rot = wbc.getEERotation();
    Eigen::Vector3d init_base_pos = wbc.getBasePosition();
    double init_base_yaw = wbc.getBaseYaw();

    init_ee_pos.z() = 0.8;

    wbc.setArmNominalPosture(wbc.getArmQ());
    wbc.setBaseTarget(Eigen::Vector2d(init_base_pos.x(), init_base_pos.y()), init_base_yaw);

    double dt = 0.0;
    double sim_time = 0.0;
    auto last_log_time = std::chrono::steady_clock::now();

    while (true)
    {
        auto loop_start = std::chrono::steady_clock::now();

        obs = mj.get_obs();
        if (!obs.qpos.empty())
        {
            wbc.update(obs);
        }

        Eigen::Vector3d target_ee_pos = init_ee_pos;
        Eigen::Vector3d target_ee_vel = Eigen::Vector3d::Zero();
        wbc.setEETarget(target_ee_pos, init_ee_rot, target_ee_vel);

        const double omega = 0.3;

        Eigen::Vector2d target_base_pos(init_base_pos.x(), init_base_pos.y());
        double target_base_yaw = init_base_yaw;

        double theta = omega * sim_time;

        target_base_yaw = init_base_yaw + theta;

        wbc.setBaseTarget(target_base_pos, target_base_yaw);

        Command cmd = wbc.solve(dt);

        mj.set_cmd(cmd);

        sim_time += dt;

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_log_time).count() >= 1.0)
        {
            last_log_time = now;
            Eigen::Vector3d curr_ee = wbc.getEEPosition();
            Eigen::Vector3d curr_base = wbc.getBasePosition();

            double curr_base_yaw = wbc.getBaseYaw();
            double pos_err_norm = (target_ee_pos - curr_ee).norm();
            double base_err_norm = (target_base_pos -  Eigen::Vector2d{curr_base.x(),curr_base.y()}).norm();
            double yaw_err = target_base_yaw - curr_base_yaw;
            while (yaw_err > M_PI)
                yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI)
                yaw_err += 2.0 * M_PI;
            std::cout << std::fixed << std::setprecision(3)
                      << "[t=" << sim_time << "s] "
                      << "EE Err: " << std::setprecision(2) << pos_err_norm * 1000.0 << " mm | "
                      << "BasePos Err: " << std::setprecision(2) << base_err_norm * 1000.0 << " mm | "
                      << "Yaw Err:" << yaw_err<< " | "
                      << std::endl;
        }

        std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - loop_start;
        dt = elapsed.count();
    }
}