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

// ---------------------------------------------------------------------------
// 演示场景（硬编码，无命令行参数）
//
//   末端目标：世界坐标系下的固定点（不随底盘移动/旋转）
//   底盘目标：保持 x/y 不动，以恒定角速度原地旋转
//
// 意义：底盘在转，而末端必须钉在世界上不动 —— 机械臂要反过来补偿底盘的旋转，
//       这是移动操作里最典型的"浮动基 + 定点作业"测试。
//
// 关于 yaw 速率的说明（实测）：本 MuJoCo 模型是 4 轮 skid-steer，原地转向要克服
// 四轮横向侧滑的阻力力偶（4*mu*N*|x| = 345 Nm @ mu=0.5），而 35 Nm/轮的驱动上限
// 只能产生 508 Nm；μ=1.0 时（689 Nm）物理上转不动。降到 μ=0.5 后（scene.xml 显式
// 指定地板摩擦）可控上限约 0.25 rad/s：0.15 → 跟随 92%，0.2 → ~90%，0.3 → 85%
// 且落在黏滑跳变区。故此处取 0.2 rad/s。
// ---------------------------------------------------------------------------
namespace
{
    constexpr double kBaseYawRate = 0.2; // rad/s，底盘恒定原地旋转角速度

    // 末端目标相对「底盘初始位姿」的世界系偏移。水平距离取 0.35 m：底盘转一整圈，
    // 目标到臂肩的距离恒定约 0.41 m（UR5 伸展极限约 0.93 m），任何朝向下都可达。
    const Eigen::Vector3d kEeTargetOffset(0.55, 0.10, 0.55);
} // namespace

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
    // 纯位置任务 + 关闭姿态任务：保持"初始零位姿态"会撞上 UR5 的腕部奇异
    // (wrist_2≈0 时失去一个转动自由度)，实测末端误差会随底盘转角累积到 180 mm+。
    config.kp_posture = 0.0;
    config.kd_posture = 0.0;

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

    // 等模型落稳再抓初始位姿：MJCF 里 chassis 初始 z=0，轮子先陷入地面约 26 mm
    // 再被弹起（实测 0.5 s 内 z 就到 0.0259 并稳定）。若用"未落稳"的状态当目标，
    // 底盘一开始就在追一个假的初始位姿。仿真按实时速率运行，故按墙钟等 1.2 s。
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

    // 世界系定点：位置和姿态都在初始化时定下，之后永不改变。
    const Eigen::Vector3d target_ee_pos = init_base_pos + kEeTargetOffset;
    const Eigen::Matrix3d target_ee_rot = wbc.getEERotation();
    const Eigen::Vector3d target_ee_vel = Eigen::Vector3d::Zero();

    // 底盘目标位置固定（原地旋转），只有 yaw 随时间推进。
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

        // 世界系定点：每拍都发同一个目标。
        wbc.setEETarget(target_ee_pos, target_ee_rot, target_ee_vel, /*use_orientation=*/false);

        // 底盘：位置不动，yaw 匀速推进。角速度作为前馈传入，否则斜坡参考必然滞后。
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

            // 用 MuJoCo 自己的正运动学对照控制器的 FK：任何状态映射回归会立刻暴露。
            double fk_gap = -1.0;
            if (obs.ee_pos.size() == 3)
            {
                fk_gap = (curr_ee - Eigen::Vector3d(obs.ee_pos[0], obs.ee_pos[1], obs.ee_pos[2])).norm();
            }

            std::cout << std::fixed << std::setprecision(3)
                      << "[t=" << sim_time << "s] "
                      << "EE Err: " << std::setprecision(2) << pos_err_norm * 1000.0 << " mm | "
                      << "BasePos Err: " << std::setprecision(2) << base_err_norm * 1000.0 << " mm | "
                      << "Yaw Err:" << yaw_err << " | "
                      << "FK gap:" << fk_gap * 1000.0 << " mm | "
                      << "[diag] |v_ee|=" << std::setprecision(3) << wbc.getEEVelocity().norm()
                      << " |qdot_arm|=" << wbc.getArmQDot().norm()
                      << " yaw_act=" << curr_base_yaw
                      << std::endl;
        }

        std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - loop_start;
        dt = elapsed.count();
    }
}
