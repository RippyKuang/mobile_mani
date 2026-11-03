import mujoco
import numpy as np

# 分别编译两个模型
m_orig = mujoco.MjModel.from_xml_path("ridgeback_ur5/urdf/ridgeback_ur5.urdf")
m_rbdl = mujoco.MjModel.from_xml_path("ridgeback_ur5/mjcf/ridgeback_ur5.xml")

d_orig = mujoco.MjData(m_orig)
d_rbdl = mujoco.MjData(m_rbdl)

# 1. 检查各 Link 的等效主惯性矩差异
inertia_diff = np.max(np.abs(m_orig.body_inertia - m_rbdl.body_inertia))
print(f"主惯性矩最大绝对误差: {inertia_diff:.2e}")

# 2. 检查质心位置差异
ipos_diff = np.max(np.abs(m_orig.body_ipos - m_rbdl.body_ipos))
print(f"质心坐标最大绝对误差: {ipos_diff:.2e}")

# 3. 计算并对比零位姿下的关节空间质量矩阵 M(q)
mujoco.mj_forward(m_orig, d_orig)
mujoco.mj_forward(m_rbdl, d_rbdl)

M_orig = np.zeros((m_orig.nv, m_orig.nv))
M_rbdl = np.zeros((m_rbdl.nv, m_rbdl.nv))
mujoco.mj_fullM(m_orig, d_orig, M_orig)
mujoco.mj_fullM(m_rbdl, d_rbdl, M_rbdl)

M_diff = np.max(np.abs(M_orig - M_rbdl))
print(f"质量矩阵 M(q) 最大绝对误差: {M_diff:.2e}")