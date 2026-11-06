#include "mj_adapter.h"

MjAdapter::MjAdapter(const char *fn)
{
    instance_ = this;
    mjcb_control = &MjAdapter::cb;
    mj_thread = std::thread(&start_simulate, fn);
}

void MjAdapter::cb(const mjModel *m, mjData *d)
{
    Observation obs;
    obs.qpos.assign(d->qpos, d->qpos + m->nq);
    obs.qvel.assign(d->qvel, d->qvel + m->nv);
    instance_->curr_obs.write(obs);

    Command cmd;
    instance_->curr_cmd.read(cmd);

    if (!cmd.ctrl.empty())
        for (int i = 0; i < m->nu; i++)
        {
            d->ctrl[i] = instance_->alpha * cmd.ctrl[i] + (1 - instance_->alpha) * d->ctrl[i];
        }
}

Observation MjAdapter::get_obs()
{
    Observation obs;
    curr_obs.read(obs);
    return obs;
}

void MjAdapter::set_cmd(const Command &cmd)
{
    curr_cmd.write(cmd);
}

MjAdapter::~MjAdapter()
{
    mj_thread.join();
}
