struct WheelContactForces {
    double fz[4]; // 4轮法向垂直力 (N)
    double fx[4]; // 4轮纵向推力 (N)
    double fy[4]; // 4轮横向摩擦力 (N, 带正负方向)
    double Mz_resist; // 合成的底盘总阻转力矩 (Nm)
};

WheelContactForces computeChassisContactForces(
    double Fx, double Mz, double Fz, double omega_z,
    double W, double L, double mu,
    double xc, double yc, double h) 
{
    WheelContactForces out;
    const double Fz_base = Fz / 4.0;
    const double pitch_transfer = (h * Fx) / (4.0 * L); // 动态俯仰载荷转移

    // 1. 基于 CoM 计算四轮法向力 fz (加 max 防止离地时为负)
    out.fz[0] = std::max(0.0, Fz_base * (1.0 + xc / L + yc / W) - pitch_transfer); // 左前 LF
    out.fz[1] = std::max(0.0, Fz_base * (1.0 + xc / L - yc / W) - pitch_transfer); // 右前 RF
    out.fz[2] = std::max(0.0, Fz_base * (1.0 - xc / L + yc / W) + pitch_transfer); // 左后 LR
    out.fz[3] = std::max(0.0, Fz_base * (1.0 - xc / L - yc / W) + pitch_transfer); // 右后 RR

    // 2. 计算四轮纵向力 fx
    out.fx[0] = Fx / 4.0 - Mz / (4.0 * W);
    out.fx[1] = Fx / 4.0 + Mz / (4.0 * W);
    out.fx[2] = Fx / 4.0 - Mz / (4.0 * W);
    out.fx[3] = Fx / 4.0 + Mz / (4.0 * W);

    // 3. 计算自转滑移方向 (tanh 平滑)
    double dir_factor = std::tanh((omega_z * L) / 0.05);

    // 4. 逐轮计算横向力 fy
    out.Mz_resist = 0.0;
    for (int i = 0; i < 4; ++i) {
        double f_limit = mu * out.fz[i];
        if (f_limit <= 1e-3) {
            out.fy[i] = 0.0;
            continue;
        }

        double rho_i = std::abs(out.fx[i]) / f_limit;
        double factor = (rho_i < 1.0) ? std::sqrt(1.0 - rho_i * rho_i) : 0.0;
        factor = std::max(0.15, factor); // 保底系数

        double fy_mag = f_limit * factor;

        // 前轮阻力为负，后轮阻力为正
        if (i == 0 || i == 1) {
            out.fy[i] = -dir_factor * fy_mag;
            out.Mz_resist += L * out.fy[i]; // 前轮产生阻转力矩
        } else {
            out.fy[i] = +dir_factor * fy_mag;
            out.Mz_resist -= L * out.fy[i]; // 后轮产生阻转力矩
        }
    }

    return out;
}