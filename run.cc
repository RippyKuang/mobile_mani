#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <atomic>
#include <iomanip>

#include <filesystem>
#include <rtplot/rtplot.hpp>
#include "mj_adapter.h"
#include "wbc.h"

namespace
{
    constexpr double kBaseYawRate = 0.5; // rad/s
    const Eigen::Vector3d kEeTargetOffset(0.55, 0.10, 0.55);
}

int main()
{

    rtplot::LoggerConfig cfg;
    cfg.persist = false;    
    cfg.shmPublish = true; 
    cfg.flushIntervalMs = 2;
    cfg.flushOnSignal = true;

    auto &log = rtplot::Logger::instance();
    if (!log.start(cfg))
    {
        std::fprintf(stderr, "logger start failed\n");
        return 1;
    }

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

    config.kp_posture = 0.0;
    config.kd_posture = 0.0;

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

    {
        const auto settle_start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - settle_start).count() < 1.2)
        {
            obs = mj.get_obs();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "[run] Model settled at z=" << std::fixed << std::setprecision(4)
                  << obs.qpos[2] << " m" << std::endl;
    }

    wbc.update(obs);

    const Eigen::Vector3d init_base_pos = wbc.getBasePosition();
    const double init_base_yaw = wbc.getBaseYaw();

    const Eigen::Vector3d target_ee_pos = init_base_pos + kEeTargetOffset;
    const Eigen::Matrix3d target_ee_rot = wbc.getEERotation();
    const Eigen::Vector3d target_ee_vel = Eigen::Vector3d::Zero();

    const Eigen::Vector2d target_base_pos(init_base_pos.x(), init_base_pos.y());

    wbc.setArmNominalPosture(wbc.getArmQ());

    std::cout << "[run] EE target (world): [" << target_ee_pos.transpose() << "] m\n"
              << "[run] base target: in-place rotation at " << kBaseYawRate << " rad/s\n";

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

        wbc.setEETarget(target_ee_pos, target_ee_rot, target_ee_vel, /*use_orientation=*/false);

        const double target_base_yaw = init_base_yaw + kBaseYawRate * sim_time;
        wbc.setBaseTarget(target_base_pos, target_base_yaw, kBaseYawRate, 0.0);

        Command cmd = wbc.solve(dt);

        mj.set_cmd(cmd);

        sim_time += dt;

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_log_time).count() >= 1.0)
        {
            last_log_time = now;
            const Eigen::Vector3d curr_ee = wbc.getEEPosition();
            const Eigen::Vector3d curr_base = wbc.getBasePosition();
            const double curr_base_yaw = wbc.getBaseYaw();

            double pos_err_norm = (target_ee_pos - curr_ee).norm();
            double base_err_norm = (target_base_pos - Eigen::Vector2d{curr_base.x(), curr_base.y()}).norm();
            double yaw_err = target_base_yaw - curr_base_yaw;
            while (yaw_err > M_PI)
                yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI)
                yaw_err += 2.0 * M_PI;

            log.log("error/base_pos", base_err_norm * 1000.0, now);
            log.log("error/yaw", yaw_err * 1000.0, now);
            std::cout << std::fixed << std::setprecision(3)
                      << "[t=" << sim_time << "s] "
                      << "EE Err: " << std::setprecision(2) << pos_err_norm * 1000.0 << " mm | "
                      << "BasePos Err: " << std::setprecision(2) << base_err_norm * 1000.0 << " mm | "
                      << "Yaw Err:" << yaw_err<< " | "
                      << "Yaw Rate Err:" << kBaseYawRate - wbc.getBaseAngularVelocity()
                      << std::endl;
            log.flush();
            
        }


        std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - loop_start;
        dt = elapsed.count();
    }
}
