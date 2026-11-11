#pragma once

#include <thread>
#include <vector>
#include <atomic>
#include "core.h"

struct Observation
{
    std::vector<double> qpos;
    std::vector<double> qvel;
    // End-effector (ur_arm_wrist_3_link) position in the world frame, straight from
    // MuJoCo. Used only to cross-check the RBDL forward kinematics inside the WBC.
    std::vector<double> ee_pos;
};

struct Command
{
    std::vector<double> ctrl;
};

template <typename T>
class Buffer
{
public:
    Buffer() : write_idx_(0), read_idx_(1), ready_idx_(2), has_new_(false) {}

    void write(const T &val)
    {
        buffers_[write_idx_] = val;

        write_idx_ = ready_idx_.exchange(write_idx_, std::memory_order_acq_rel);
        has_new_.store(true, std::memory_order_release);
    }

    void read(T &val)
    {
        if (has_new_.exchange(false, std::memory_order_acquire))
        {
            read_idx_ = ready_idx_.exchange(read_idx_, std::memory_order_acq_rel);
        }
        val = buffers_[read_idx_];
    }

private:
    T buffers_[3];
    int write_idx_;
    int read_idx_;
    std::atomic<int> ready_idx_;
    std::atomic<bool> has_new_;
};

class MjAdapter
{
public:
    MjAdapter(const char *fn);
    ~MjAdapter();

    Observation get_obs();
    void set_cmd(const Command &cmd);

    MjAdapter(const MjAdapter&) = delete;
    MjAdapter& operator=(const MjAdapter&) = delete;

private:
    static inline MjAdapter *instance_ = nullptr;
    static void cb(const mjModel *m, mjData *d);

    double alpha = 0.8;

    Buffer<Observation> curr_obs;
    Buffer<Command> curr_cmd;

    std::thread mj_thread;
};