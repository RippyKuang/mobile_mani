# 移动操作机器人「跟踪效果差」排查手册

> 一份可以**不依赖 AI** 自行定位同类问题的实战教程。
> 全部结论来自 Ridgeback + UR5（12/16 自由度 WBC + 分级 QP + MuJoCo 仿真）的真实排查过程，
> 文中每个数字都是实测值，不是估计。

---

## 目录

- [0. 这份文档解决什么问题](#0-这份文档解决什么问题)
- [1. 心法：把「跟踪不好」拆成五层](#1-心法把跟踪不好拆成五层)
- [2. 第 1 层：状态与索引](#2-第-1-层状态与索引)
- [3. 第 2 层：模型一致性](#3-第-2-层模型一致性)
- [4. 第 3 层：优化层（QP）](#4-第-3-层优化层qp)
- [5. 第 4 层：执行映射](#5-第-4-层执行映射)
- [6. 第 5 层：物理可行性](#6-第-5-层物理可行性)
- [7. 完整案例复盘（含 5 条弯路）](#7-完整案例复盘含-5-条弯路)
- [8. 物理与运动学速查卡](#8-物理与运动学速查卡)
- [9. 代码片段库（可直接复用）](#9-代码片段库可直接复用)
- [10. 症状 → 原因速查表](#10-症状--原因速查表)
- [11. 附录：本次全部实测数据](#11-附录本次全部实测数据)

---

## 0. 这份文档解决什么问题

**典型症状**：

- 机械臂跟得挺好，但底盘 x/y/yaw 跟不住
- 误差不收敛，或者收敛到某个平台期后线性累积
- 有时候能动、有时候像死了一样
- 调增益越调越糟，加积分就发散

**核心结论（先记住这句）**：

> **"跟踪不好"有 90% 的概率不是增益问题。**
> 在动 PID 参数之前，请先按顺序排掉：状态错、模型错、优化错、映射错、物理不可达。
> 这五件事任一件错了，调增益都是白费，而且会把你往错误的假设上带得更远。

---

## 1. 心法：把「跟踪不好」拆成五层

控制系统是一条链，**任何一环断了，现象都长得像"跟踪不好"**。所以排查的第一步不是看现象，而是**沿链定位**：

```
┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
│ 1 状态/索引 │──▶│ 2 模型 M,h,J│──▶│ 3 优化(QP)  │──▶│ 4 执行映射  │──▶│ 5 物理可行性│
│ 我看到的 q  │   │ 我的动力学  │   │ 我解出的 qdd│   │ 我发出的力矩│   │ 力真的够吗  │
└─────────────┘   └─────────────┘   └─────────────┘   └─────────────┘   └─────────────┘
```

### 三个"一次实验就能定生死"的判据

在深入任何一层之前，先做这三个实验，通常 30 分钟内能把范围缩到一层：

| 实验 | 做法 | 判定 |
|---|---|---|
| **A. 任务解耦** | 暂停其中一个任务，只留另一个 | 谁跟得上、谁跟不上，立刻知道断链在哪一侧 |
| **B. FK 交叉校验** | 用仿真器的正运动学对照控制器内部 FK | 差 >1 cm ⇒ 状态/索引层（第 1 层）有问题 |
| **C. 开环扫描** | 给恒定输入，测稳态输出，画输入-输出曲线 | 曲线是"线性"还是"死区+陡坡"⇒ 决定第 5 层是不是瓶颈 |

本次排查就是靠 **A** 起步的：

```
日志：[t=10s] EE Err: 20.48 mm | BasePos Err: 362.01 mm | Yaw Err: 1.23
                    ↑ 好                       ↑ 差            ↑ 差
```

> **结论直接得出**：EE 任务（当时是 level 0）跟得很好 ⇒ 状态映射、模型、QP、物理在"机械臂这条链"上都是通的。
> 问题必然在**底盘这条链**上。这一步省掉了 80% 的排查范围。

**教训 A**：永远先找"能工作的对照项"。EE 好而底盘差，比"全都乱"能给出多得多的信息。

---

## 2. 第 1 层：状态与索引

### 2.1 为什么这层最容易错

浮基 + 四元数 + 多关节 = 索引地狱。而且**错了不会报错**，只会让控制器拿到看似合理的错误数据，然后表现为"跟踪不好"。

### 2.2 三个必须记住的约定

**① RBDL 3.x 的四元数 `w` 分量追加在 `q` 的末尾**

不是紧跟在 `xyz` 后面！源码 `rbdl/src/Model.cc:386-399`：

```c
// update the w components of the Quaternions. They are stored at the end of the q vector
multdof3_w_index[i] = dof_count + multdof3_joint_counter;
...
q_size = dof_count + multdof3_joint_counter;
```

所以本项目 12 自由度模型的实际布局是：

```
q[0..2]   = 基座 xyz
q[3..5]   = 四元数 x,y,z
q[6..11]  = 6 个臂关节
q[12]     = 四元数 w      ← 在最后！
q_size = 13, dof_count = 12, qdot_size = 12
```

**验证方法**（千万别靠猜）：

```cpp
// init() 里打印 + 断言，一劳永逸
if (model_->dof_count != 12 || model_->q_size != 13 || model_->qdot_size != 12) {
    std::cerr << "unexpected: dof=" << model_->dof_count
              << " nq=" << model_->q_size << " nv=" << model_->qdot_size << "\n";
    return false;
}
```

**② 角速度的坐标系**

| 来源 | 线速度 | 角速度 |
|---|---|---|
| MuJoCo `qvel[0..5]`（free joint） | 世界系 | **body 局部系** |
| RBDL `JointTypeSpherical` 的 qdot | — | **body 局部系** |

两者恰好一致，可以直接拷贝。但如果你把它当世界系用（或反过来），**位置看起来还是对的**，只有 `h(q,qdot)` 里的科氏项会错——极难发现。

判断依据（MuJoCo 官方文档 `doc/overview.rst`）：

> The linear positions of free joints are in the global frame, as are linear velocities.
> However, the rotational velocities of a free joint are in the local body frame.

**③ 不要写死索引！**

原代码里到处是 `q_[10 + i]`、`VectorXd::Constant(16, ...)`。改一次模型维度，16 处全部要改，漏一处就静默出错。正确做法：

```cpp
// 按名字解析，然后在 init 里断言布局
std::array<int, 6> arm_q_idx_{}, arm_dof_idx_{};
static const char *arm_link_names[6] = {
    "ur_arm_shoulder_link", "ur_arm_upper_arm_link", "ur_arm_forearm_link",
    "ur_arm_wrist_1_link", "ur_arm_wrist_2_link", "ur_arm_wrist_3_link"};
for (int i = 0; i < 6; ++i) {
    const unsigned int body_id = model_->GetBodyId(arm_link_names[i]);
    arm_q_idx_[i] = static_cast<int>(model_->mJoints[body_id].q_index);
    arm_dof_idx_[i] = arm_q_idx_[i];       // 1-DoF 关节且位于浮基之后
    if (arm_q_idx_[i] != 6 + i) {          // 布局不合预期就直接拒绝启动
        std::cerr << "unexpected q_index layout for " << arm_link_names[i] << "\n";
        return false;
    }
}
```

### 2.3 终极检查：FK 交叉校验（本次最有价值的一招）

**做法**：让仿真器也报一份末端位姿，和控制器的 FK 对比。

```cpp
// simulate/mj_adapter.cc —— 在控制回调里把 MuJoCo 的 FK 结果塞进观测
struct Observation { std::vector<double> qpos, qvel, ee_pos; };

void MjAdapter::cb(const mjModel *m, mjData *d) {
    static int ee_body_id = -2;
    if (ee_body_id == -2) ee_body_id = mj_name2id(m, mjOBJ_BODY, "ur_arm_wrist_3_link");

    Observation obs;
    obs.qpos.assign(d->qpos, d->qpos + m->nq);
    obs.qvel.assign(d->qvel, d->qvel + m->nv);
    if (ee_body_id >= 0) {
        mj_kinematics(m, d);   // mjcb_control 在 forward 之前调用，需手动刷新
        obs.ee_pos.assign(d->xpos + 3*ee_body_id, d->xpos + 3*ee_body_id + 3);
    }
    instance_->curr_obs.write(obs);
    // ...
}
```

```cpp
// run.cc —— 每秒打印一次 gap
double fk_gap = (wbc.getEEPosition() - Eigen::Vector3d(obs.ee_pos[0], obs.ee_pos[1], obs.ee_pos[2])).norm();
std::cout << "FK gap:" << fk_gap * 1000.0 << " mm";
```

**判定标准**：

- `0.00 mm` ⇒ 状态映射正确，永远信任它
- `mm 级` ⇒ 可能是 URDF/MJCF 尺寸差异（去第 2 层）
- `cm 级` ⇒ **索引错了**，立刻停下排查
- `m 级` ⇒ 完全错（比如 base 和 EE 搞混）

本项目实测 `0.00 mm`。**这条自检建议永久保留**——任何索引回归都会在 1 秒内暴露。

---

## 3. 第 2 层：模型一致性

### 3.1 症状

前馈不灵、需要巨大的积分项才能动、增益怎么调都不对、EE 稳态偏差大。

### 3.2 核心方法：双模型对拍

**思路**：让控制器用的模型（URDF→RBDL）和仿真用的模型（MJCF→MuJoCo）都走**同一个引擎**编译，然后逐项对比。这样能把"模型差异"和"引擎差异"分开。

```python
# tools/validate.py 的核心：把两边都化简成同一个 12 自由度模型再比
WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]

def build_rbdl_equivalent(urdf_path):
    spec = mujoco.MjSpec.from_file(urdf_path)
    spec.compiler.fusestatic = False        # ← 关键！见 3.3
    spec.body("chassis_link").add_freejoint()
    for w in WHEELS:
        j = spec.joint(w)
        if j is not None: spec.delete(j)
    return spec.compile()

def build_sim_equivalent(scene_path):
    spec = mujoco.MjSpec.from_file(scene_path)
    for a in list(spec.actuators):
        if a.target in WHEELS: spec.delete(a)
    for w in WHEELS:
        j = spec.joint(w)
        if j is not None: spec.delete(j)
    return spec.compile()
```

对比项与阈值：

| 项 | 阈值 | 说明 |
|---|---|---|
| 总质量 | < 1e-3 kg | 最容易被打破的一项 |
| 质心 | < 1 mm | |
| `M(q)` 相对误差 | < 1% | 多取几个构型 |
| `h(q)` 绝对误差 | < 1 N/Nm | 零速下的重力项 |
| 末端位置 | < 2 mm | |
| 末端雅可比 | < 1% | |

### 3.3 五个必踩的坑（本次全踩了）

#### 坑 ① `fusestatic` 让 URDF 校验变成假的

**现象**：脚本"通过"了，但模型其实是错的。

**原因**：MuJoCo 编译 URDF 时 `compiler.fusestatic` 默认为 `true`，会把**没有关节的根 link 焊进 world 并丢弃其惯量**。

实测（本项目旧版 `tools/validate.py`）：

```
nv = 10           ← 应该是 16
总质量 = 26.19 kg ← 应该是 220.26 kg（194 kg 底盘凭空消失）
```

它比的是"只有机械臂的两个模型"，所以怎么比都对。

**修正**：`spec.compiler.fusestatic = False` + 给根 link 补 `add_freejoint()`。

> **教训**：任何"对拍通过"的结论，都要先打印**双方的维度（nv/nq）和总质量**。数字对不上就是剧本错了。

#### 坑 ② RBDL 只读 `<inertial>`，MuJoCo 还会按 collision geom 补质量

URDF 里没有 `<inertial>` 的 link：
- RBDL：**零质量**
- MuJoCo：按 collision mesh + density 推算出一个质量

本项目 `top_link` 有 collision mesh 但没 `<inertial>` ⇒ MuJoCo 补了 **4.229 kg**，RBDL 补 0 ⇒ 总质量差 1.9%。

**检查方法**（写进 validate.py）：

```python
def check_inertial_coverage(urdf_path):
    root = ET.parse(urdf_path).getroot()
    for link in root.findall("link"):
        if link.find("inertial") is None and link.find("collision") is not None:
            print(f"FAIL '{link.get('name')}' has collision but no <inertial>")
```

**修正**：给 `top_link` 补上惯量（数值从 MuJoCo 的网格推算结果取）⇒ 总质量误差从 1.9% 降到 **2.9e-4 kg**。

#### 坑 ③ MuJoCo 的接触摩擦按元素取 `max()`，会悄悄覆盖你的设定

**现象**：MJCF 里轮子明明写了 `friction="0.5 0.005 0.02"`，但实际行为像 μ=1.0。

**原因**（`mujoco/src/engine/engine_collision_driver.c:1475`）：

```c
for (int i = 0; i < 3; i++) fri[i] = mju_max(friction1[i], friction2[i]);
```

地板是默认 `friction="1 0.005 0.0001"`，轮子是 0.5 ⇒ **max ⇒ 1.0**。

**一行验证**：

```python
m = mujoco.MjModel.from_xml_path("scene.xml")
print(m.geom_friction[0])          # 地板
# 有效 mu = max(floor, wheel)
```

**修正**：地板**显式**写 `friction="0.5 0.005 0.0001"`。

> **教训**：仿真里凡是有"两个物体都声明了同一个参数"的地方，都要去查**合成规则**。摩擦、solref、solimp、priority 全是。

#### 坑 ④ URDF 关节缺 `<axis>` ⇒ 默认 (1,0,0)

本项目 URDF 的 4 个轮子关节：

```xml
<joint name="front_left_wheel" type="revolute">
  <parent link="front_rocker_link"/>
  <child link="front_left_wheel_link"/>
  <origin rpy="0 0 0" xyz="0 0.2755 0"/>
</joint>
<!-- 没有 <axis>！URDF 规范默认 1 0 0 -->
```

而 MJCF 里写的是 `axis="0 1 0"`。**RBDL 里的轮子绕 x 转，仿真里绕 y 转** ⇒ `M_`、`h_` 的轮子耦合项全错。

**验证方法**：让 MuJoCo 的 URDF 导入器告诉你它读到什么。

```python
mu = mujoco.MjModel.from_xml_path("robot.urdf")
for j in range(mu.njnt):
    print(mujoco.mj_id2name(mu, mujoco.mjtObj.mjOBJ_JOINT, j), mu.jnt_axis[j])
```

输出 `front_left_wheel [1. 0. 0.]` ⇒ 确认。

> **教训**：**同一个机器人的两套描述文件（URDF/MJCF）不能靠"看起来一样"来判断一致**，必须机器对拍。

#### 坑 ⑤ 惯量张量的坐标轴搞混

URDF 的 `<inertial><origin rpy>` 和 `<inertia>` 是两套坐标。若 rpy 非零而惯量没做旋转补偿，M(q) 会错。

**检查**：对拍时出现的误差如果**只集中在角加速度行**，通常是这个原因。

### 3.4 「去掉不激励的自由度」这一招

**问题**：轮子自由度让模型变复杂，而且（本项目）轮子轴还写错了。

**解法**（本项目采用的）：把 URDF 里 4 个轮子关节从 `revolute` 改成 `fixed`。

- RBDL 会把轮子惯量按平行轴定理合并进 `chassis_link` ⇒ 质量不丢
- 模型降到 **12 自由度**（浮基 6 + 臂 6），正好等于真正需要求解的量
- 顺带消灭了坑 ④（轮子自由度不存在了，轴错也没关系）
- 语义更干净：`tau_gen[0..5]` 变成**整车外部力旋量**，正好是轮子该提供的东西

**代价**：模型里没有轮子动力学了，轮速只能从仿真遥测读（本次保留了 `wheel_qdot_` 作为遥测）。

**结果**：`M(q)` 相对误差 4.5e-4、`h(q)` 差 2.8e-3 N、EE FK 0.00 mm。

---

## 4. 第 3 层：优化层（QP）

### 4.1 症状

- 行为"时好时坏"、周期性呆滞
- 有时像完全没有控制器在工作
- 日志里有 `maximum iterations reached`

### 4.2 三个必查项

#### 查 ① 失败率 + 失败后干了什么

原代码（`hqp_solver.cc`）：

```cpp
if (!ok) {
 // std::cerr << "[HQPSolver] Failed to solve priority level " << lvl << std::endl;  ← 被注释掉了！
    if (lvl == 0) return false;
    break;      // ← level-1 失败就静默丢弃整个底盘任务
}
```

而调用侧：

```cpp
bool success = hqp_solver_->solve(qddot_opt);
if (!success) qddot_opt = Eigen::VectorXd::Zero(nv_);   // ← 失败就发零加速度
```

**后果**：底盘任务大量周期根本没生效，而且日志里看不出来。

**实测**：修复前 21 秒内 **≥5500 次 QP 失败**（日志按每 500 次打印一条，出现了 12 条）。

**修复**：打印带层号 + 降级策略改为"沿用上一拍解"。

#### 查 ② `|b|` 和变量界的量级比

这是**判断 QP 是否病态的最快方法**：

```
ratio = max|b_task| / max|变量上界|
```

- ratio < 1 ⇒ 健康，QP 可以精确追踪
- ratio > 10 ⇒ 任务永远饱和，本质是 bang-bang
- ratio > 100 ⇒ slack 惩罚会主导代价函数，必然 max_iter

本项目原代码：

```cpp
// 底盘任务的 b 是「加速度」，但增益是这么给的：
b_base[0] = config_.kp_base * yaw_diff + config_.ki_base * yaw_err_integral_;
//           kp_base = 5000 (!)              ki_base = 500
// 而变量上界 qdd_max = 40
// ratio ≈ 5000*0.1/40 = 12.5（比例项）  积分饱和后 = 5000/40 = 125
```

**教训**：`b` 是加速度就必须用加速度量纲的增益（1/s²），不能拿位置误差乘 5000。

#### 查 ③ 分级走廊的设计

分级 QP 常用"低优先级任务必须把高优先级任务的达成值保持在 ±δ 内"：

```cpp
b_eq_lb = achieved - delta;
b_eq_ub = achieved + delta;
```

**陷阱**：固定的 `delta = 0.05` 对所有任务不公平。

- 对 EE 加速度任务（量级 ~10 m/s²）：0.05 是 0.5% —— 极紧
- 对底盘加速度任务（量级 ~2 rad/s²）：0.05 是 2.5% —— 还行
- 如果某任务的 b 是 5000，0.05 就是 1e-5 —— **完全不可行**

**修正**：把容差变成**每个任务自己的属性**：

```cpp
struct HQPTask {
    std::string name;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    double weight = 1.0;
    double tolerance = 0.05;   // 在这个任务的单位下，传给下一级的走廊宽度
};
```

调用时按量纲给：EE 任务 0.5（m/s²），底盘任务 0.1，姿态任务 0.1。

### 4.3 OSQP 调参要点

```cpp
settings->eps_abs = 1e-4;      // 太松 → 解不准；太紧 → max_iter
settings->eps_rel = 1e-4;
settings->max_iter = 2000;
settings->adaptive_rho = 1;    // OSQP 1.0 里这是「自适应方法」枚举，0 = 关闭
settings->polishing = 0;       // 注意：OSQP 1.0 会无视 verbose 往 stdout 打 "Polishing not needed"
```

**关于 `rho_`（slack 惩罚）**：

- 太小 ⇒ 高优先级任务被牺牲
- 太大 ⇒ 代价函数被 slack 主导，与 `b` 的量级不匹配就必然 max_iter
- 经验值：**与任务 b 的量级同阶**。本项目 1e4 → 1e3

**效果**：失败从 ≥5500 次/21s → **1–2 次/16s**。

### 4.4 性能：不要每拍都 setup/cleanup

原代码每个控制周期创建 + 销毁 3 个 `OSQPSolver`（含符号分解）。正确做法是按 `(n_var, n_constr)` 缓存求解器，用 `osqp_update_data` 更新。本次未做（失败率已降到 0.01%），但高频控制时这是第一优化项。

---

## 5. 第 4 层：执行映射

### 5.1 症状

命令"看起来对"，实际行为反了、或者只有一半、或者时有时无。

### 5.2 坑 ① 先合成后 clamp 会摧毁差动 ⭐

**这是本次最隐蔽、破坏力最大的 bug。**

原代码：

```cpp
double tau_fwd = (Fx / 4.0) * wheel_radius;
double tau_yaw = (Mz / (4.0 * track_half_width)) * wheel_radius;

double fl = tau_fwd - tau_yaw;
double fr = tau_fwd + tau_yaw;
// ...
cmd.ctrl[0] = std::clamp(fl, -35, 35);      // ← 合成之后才 clamp
cmd.ctrl[1] = std::clamp(fr, -35, 35);
```

**当 `tau_fwd` 很大时**（比如 80 Nm）：

```
fl = 80 - 25 = 55  → clamp → 35
fr = 80 + 25 = 105 → clamp → 35
                              ↑ 左右变成了同一个值！
```

**差动被完全抹掉 ⇒ 不转向，只往前冲。**

**数值链条**（本项目实测）：

```
底盘任务索引错位 → x 误差喂给了「世界系横向力」(TY) 自由度
  → b = 5000 × 0.2 = 1000 m/s² → 被限到 40
  → tau_gen[1] = m_total × 40 = 220 × 40 ≈ 8800 N
  → Fx = 8800 × sin(yaw)          ← 只有 sin(yaw) 那一小部分能进前向通道
  → tau_fwd = 8800 × sin(yaw) / 4 × 0.0759 ≈ 167·sin(yaw) Nm
  → yaw > 0.2 rad 后四轮全部同向饱和 → 差动归零
```

**修正**：

```cpp
// 分项 clamp，且保证差动优先拿到预算
const double limit   = std::min(tau_wheel_max, traction_torque);
const double yaw_cap = std::min(limit, tau_diff_max);
tau_yaw = std::clamp(tau_yaw, -yaw_cap, yaw_cap);

const double room_fwd = std::max(0.0, limit - std::abs(tau_yaw));
tau_fwd = std::clamp(tau_fwd, -room_fwd, room_fwd);

// 现在再合成，两个值必然都在预算内
double fl = tau_fwd - tau_yaw, fr = tau_fwd + tau_yaw;
```

### 5.3 坑 ② 「广义力」不等于「执行器力矩」

经典错误：算出 `tau_gen = M*qdd + h` 后，直接把 `tau_gen[0]`（平动自由度）当"推力"，再除以 4 分给轮子。

问题：
- 平动自由度是**世界系**的，轮子只能产生**车体系前向力**
- 横向分量会被静默丢弃（差速底盘本来就做不到）
- 轮子自由度自己的广义力（`tau_gen[6..9]`）被完全浪费

**判断方法**：问自己"我这个映射，在什么前提下才成立？" —— 前提不成立时（比如车已经转过 90°），映射就失效。

### 5.4 坑 ③ 位置环和转向环抢同一个预算

差速底盘四个轮子的力矩预算（受电机上限 + 摩擦锥限制）是**共享**的。

本项目 μ=0.5 时牵引力上限只有 `μ·N·r = 0.5 × 540 × 0.0759 = 20.5 Nm/轮`。如果位置环因为漂移而要求大的前向力，差动就没有余量 ⇒ 转向停滞。

**修正**：**按任务优先级分配预算**。既然参考是偏航斜坡，就让偏航先拿，剩余给位置环。

```cpp
// 偏航优先（配合「偏航优先、接受漂移」的策略）
tau_yaw = clamp(tau_yaw, ±yaw_cap);
room_fwd = limit - |tau_yaw|;
tau_fwd  = clamp(tau_fwd, ±room_fwd);
```

### 5.5 检查方法：开环标定表

不要猜映射系数对不对，**直接测**：

```python
for ty in (5, 10, 15, 20, 25, 30):
    d = mujoco.MjData(m); ...  # 施加恒定差动
    d.ctrl[FL] = -ty; d.ctrl[FR] = +ty; d.ctrl[RL] = -ty; d.ctrl[RR] = +ty
    for k in range(3000): mujoco.mj_step(m, d)
    print(ty, np.mean(qvel_z_hist))
```

得到 `τ → ω` 曲线后：
- 曲线**线性** ⇒ 映射系数可以拟合修正
- 曲线**死区 + 陡坡** ⇒ 见第 5 层

---

### 5.6 坑 ④ QP 会「作弊」：用执行器做不到的自由度 ⭐⭐

**这是本次最后才找到、但影响最大的一个问题。**

#### 症状

底盘开始转动后，末端误差从几毫米开始**随转角线性增长**（45 → 500 mm），
但机械臂关节看起来在动、QP 各层都在正常求解（只有最后一层偶尔失败）。

#### 排查过程

打印 QP 实际解出来的加速度（不要只看指令，要看优化变量）：

```
[qp] qddot_base = -17.29   0.28  -0.01   0.00   0.00   8.82
     qddot_arm  = -40.00  40.00 -21.19 -26.91  13.05   0.00
```

底盘 x 方向被要求 **-17 m/s²** 的加速度。而差速底盘的物理极限约 `μ·g ≈ 4.9 m/s²`。

#### 根因

QP 的决策变量是**全部 12 个自由度**，包括底盘的 TX/TY/TZ/RX/RY/RZ。
但差速底盘真正可控的只有 2 个方向：**沿航向的平动** 和 **偏航**。

末端任务发现：要产生 1 m/s² 的末端加速度，
- 用手臂 ⇒ 需要 ~2.5 rad/s² 的关节加速度
- 用「横向推底盘」⇒ 只需要 1 m/s² 的底盘加速度（末端在 0.4 m 高处，杠杆最大）

于是 QP 选择了后者。**它以为末端会被推到位，于是没给机械臂足够的指令；
而现实世界里那个横向加速度根本不存在，轮子只是打滑。**

#### 修正：把它作为硬约束加进每一级

横向方向 = 航向的法向 `(-sin(yaw), cos(yaw))`。注意这是 TX/TY 的**线性组合**，
用 box bound（`lb[i] <= x[i] <= ub[i]`）表达不了，必须用线性约束：

```cpp
// 每一级 QP 都要带上这条（hqp_solver->setLinearConstraints 会传进每个 level）
Eigen::MatrixXd C_nh = Eigen::MatrixXd::Zero(1, nv_);
C_nh(0, 0) = -std::sin(yaw);
C_nh(0, 1) =  std::cos(yaw);
hqp_solver_->setLinearConstraints(
    C_nh,
    Eigen::VectorXd::Constant(1, -config_.base_lateral_acc_max),  // 例如 0.5 m/s²
    Eigen::VectorXd::Constant(1,  config_.base_lateral_acc_max));
```

**效果**：末端误差从 45→500 mm 发散，变成 **0.9–30 mm**（底盘已转过 152°）。

#### 通用规则（背下来）

> **QP 的决策变量里只要包含了「执行器做不到的方向」，
> 那个方向一定会被某个任务拿去作弊。**

检查方法：
1. 打印 QP 的解（不是指令），逐个看分量的量级是否合理
2. 列出执行器的**真实可控方向集合**（差速底盘：2 个；全向轮：3 个；机械臂：6 个）
3. 对每个"决策变量有、但执行器没有"的方向，加一条零约束

这条规则和 [5.3](#53-坑--广义力不等于执行器力矩) 是同一个病的两种表现：
一个在**任务层**作弊（用不存在的自由度），一个在**映射层**作弊（把广义力当推力）。

## 6. 第 5 层：物理可行性

> **这一层是最容易被忽略、也最伤人的一层。**
> 如果目标在物理上不可达，你可以把控制器优化到极限，也依然"跟踪不好"。

### 6.1 判据：开环曲线是什么形状

```
ω ↑
  │                    ╱ 失控区（打滑/空转）
  │                 ╱
  │        ┌──────╱  ← 陡坡：增益极高，闭环无法稳定
  │   ─────┘        ← 死区：静摩擦/间隙，给了力也不动
  └──────────────────▶ 输入（力矩）
```

**只要看到这个形状，就必须回答一个问题：我的目标值落在哪一段？**

- 落在**线性段** ⇒ 控制器可以精确跟踪，去调参
- 落在**陡坡段** ⇒ 目标不可达（要么慢一档，要么跳变失控）
- 落在**死区** ⇒ 需要前馈/摩擦补偿

### 6.2 本次的实测曲线

**μ_eff = 1.0**（地板默认值生效时）：

| τ_diff | 稳态 ω | 标准差 | 漂移 |
|---|---|---|---|
| 22–25 Nm | 0.026–0.034 | 0.005（稳） | 0.13–0.27 m |
| 26 Nm | 0.099 | 0.037 | 0.39 m |
| 27 Nm | 0.558 | 0.444（极限环） | 0.86 m |
| 30 Nm | 1.123 | 0.537 | 0.57 m |

**结论**：稳定区间只到 **0.03 rad/s**，往上直接跳进 0.5–1.9 rad/s 的乱振区。参考值 0.3 落在陡坡顶端 ⇒ **物理不可达**。

**μ_eff = 0.5**（显式改地板后）：

| τ_diff | 稳态 ω | 标准差 |
|---|---|---|
| ≤13 Nm | 0.004–0.022 | ~0（稳） |
| 14 Nm | 0.232 | 0.174 |
| 16 Nm | 1.440 | 0.491 |
| 20 Nm | 5.575 | 0.283 |

**结论**：断点从 26 → 14 Nm，稳定可达上限从 0.03 → **0.25 rad/s**（8 倍）。但 0.3 仍落在 14 Nm 的陡坡上。

### 6.3 方法：先做力矩预算核算

在做任何实验之前，**用纸算一次**，能省掉大量试错。

```
驱动侧（差动轮力矩能产生多大偏航力矩）：
  M_drive = Σ (−y_i · F_xi) = 2·d_y·(F_r − F_l)
          其中 F_r − F_l = 2·τ_diff / r_w
  τ=35 Nm, d_y=0.2755, r_w=0.0759
  ⇒ M_drive = 2 × 0.2755 × (2×35/0.0759) = 508 Nm

阻力侧（四轮侧向侧滑形成的反向力偶）：
  M_resist = Σ |x_i| · μ · N = 4 × 0.319 × μ × (mg/4)
  μ=1.0 ⇒ 4 × 0.319 × 540 = 689 Nm
  μ=0.5 ⇒ 4 × 0.319 × 270 = 345 Nm
```

**508 < 689 ⇒ μ=1.0 时原地转向在物理上不可能**（无论控制器多聪明）。
**数值对上了实测现象**：μ=1.0 时给满 35 Nm 也只是空转打滑。

### 6.4 关键几何洞察：为什么圆弧也没用

很多人的直觉是"别原地转，走圆弧就没那么多打滑了"。**错。** 推导：

刚体上一点速度：`v = v_O + ω ẑ × r`，在车体系下：

```
v_x = v − ω·r_y     ← 沿滚动方向：车轮自转就能吸收 ✅
v_y =     ω·r_x     ← 垂直滚动方向：车轮是刚性的，无法吸收 ❌
```

关键：**`v_y` 只取决于 `ω·r_x`（前后偏置），跟前进速度 `v` 完全无关**。

实测（ω=0.3 rad/s）：

| 工况 | 滚动分量 v_x | 侧滑分量 v_y |
|---|---|---|
| 原地旋转 v=0 | ∓0.083 m/s | **±0.096 m/s** |
| 圆弧 v=0.15 | +0.067 / +0.233 m/s | **±0.096 m/s** ← 一模一样 |

**教训**：要想减少侧滑，必须改变的是 **`r_x`（前后偏置）**，不是轨迹形状。
把轮子挪到同一根轴（`r_x = 0`）⇒ `v_y ≡ 0` ⇒ 任意 ω 都没有侧滑。

### 6.5 常见"看起来能救命但没用"的手段（本次全部实测排除了）

| 尝试 | 结果 |
|---|---|
| `noslip_iterations = 10` | 28 Nm 下 ω: 0.78 → 0.67（没本质变化） |
| `impratio = 10` | 28 Nm 下 ω: 0.78 → 0.30（更差） |
| 换 Newton 求解器 | 完全一致 |
| 换 elliptic/pyramidal 锥 | 完全一致 |
| 速度伺服（`<velocity kv=30>`，力限 35/80/150 Nm） | ω 仍 0.028–0.06 rad/s |
| 各向异性摩擦（`<pair friction="0.05 1.0 ...">`） | 机制生效，但**切向轴是世界固定的**，车一转就失效 |
| 走圆弧轨迹 | 侧滑不变 |

### 6.6 MuJoCo 摩擦模型的真实能力边界

想用各向异性摩擦解决"滚得动、横不动"？先搞清楚 MuJoCo 支持什么：

| 入口 | 数组长度 | 语义 | 各向异性 |
|---|---|---|---|
| `<geom friction>` | 3 | `[滑动, 扭转, 滚动]`，滑动那个**同时作用于两个切向** | ❌ |
| `<pair ... friction>` | **5** | `[切向1, 切向2, 扭转, 滚动1, 滚动2]` | ✅ |

源码印证（`engine_collision_driver.c`）：

```c
// 各向同性入口：两个切向共用一个数
friction[0] = fri[0];
friction[1] = fri[0];   // ← 同一个值
```

**但** `<pair>` 的切向轴由 `mju_makeFrame()` 从**法向**推出，竖直法向下恒等于世界 `(0,1,0)`：

```python
# 实测：同一个轮子在 yaw=0° 和 yaw=90° 下
yaw =  0°:  frame = [[0,0,1], [0,1,0], [-1,0,0]]
yaw = 90°:  frame = [[0,0,1], [0,1,0], [-1,0,0]]   ← 完全一样！
```

⇒ **各向异性不跟车身转**。对会偏航的车头，这条路走不通（除非车头朝向固定）。

各向异性本身确实生效（滑动实验：各向同性 0.6 时横向滑行 0.080 m，各向异性 0.05/1.0 时 0.755 m），只是**轴不对**。

### 6.7 当物理不可达时，只有三条路

| 方案 | 做法 | 代价 | 彻底性 |
|---|---|---|---|
| **A. 改几何** | 碰撞体挪到同一根轴（`r_x=0`），视觉网格不动 | 改 MJCF 几十行 | ✅ 彻底 |
| **B. 降 μ** | 显式改地板摩擦 | 1 行 | ⚠️ 缓解（本次：0.03→0.25 rad/s） |
| **C. 换底座模型** | free joint 上加速度/速度伺服，绕开轮胎接触 | 物理真实性下降 | ✅ 彻底 |

**决策原则**：如果目标速率落在**陡坡**上，A/C 是唯一解；如果只是"想让它动起来"，B 就够。

---

## 7. 完整案例复盘（含 5 条弯路）

### 时间线

```
① 现象：EE 20 mm / BasePos 362 mm / Yaw 1.23
   └─ 实验 A（任务解耦思维的镜像）：EE 好 ⇒ 问题在底盘链路

② 读代码，找三类可疑点：索引、量纲、优先级
   ├─ 底盘任务 A_base 索引错位一格（x→TY, y→TZ 且 TZ 被 ±1e-3 卡死）
   ├─ b_base 用 kp=5000 当地加速度（量纲错 100 倍）
   ├─ 底盘任务在 level 1，EE 在 level 0（优先级反了）
   └─ 力矩映射先合成后 clamp（差动会被抹掉）

③ 开环实验（实验 C）：τ_diff → ω
   └─ 发现死区 + 陡坡 ⇒ 警觉物理层可能是瓶颈

④ 力矩预算核算
   └─ 发现 M_drive(508) < M_resist(689) ⇒ μ=1.0 转不动
      └─ 顺藤摸瓜：MuJoCo 摩擦 max 规则把 0.5 顶成了 1.0

⑤ 模型重构（12 自由度）+ 对拍脚本
   └─ 发现旧 validate.py 是假验证（fusestatic 丢了 194 kg）

⑥ 修任务：车体系 2 行（前向加速度 + 偏航角加速度）+ 量纲正确的增益 + ω 前馈 + 优先级反转

⑦ 修映射：分项 clamp + 偏航优先分配预算 + 摩擦前馈

⑧ 修 QP：per-task 容差 + rho 降档 + 失败日志带层号

⑨ 验证：直线 BasePos 2–6 mm、EE 2–9 mm、QP 失败 1–2 次/16s

⑩ yaw 仍不可达 → 系统排查物理层（见 6.5 表格）→ 降 μ → 0.03→0.25 rad/s

⑪ 换成「世界系定点末端 + 底盘原地转」后末端又发散
   └─ 打印 QP 解 → 发现底盘横向加速度被要求 -17 m/s²（物理上限 4.9）
   └─ 加非完整约束（横向加速度≈0）→ 末端 45→500mm 变成 0.9~30mm
```

### 5 条弯路（省下这些时间）

#### 弯路 1：想先调增益

**当时的想法**："跟踪不好，那肯定是 PID 没调好。"

**为什么错**：`b_base` 的量纲就错了（加速度项放位置增益 5000）。在这种前提下调增益，只是在一个错误的参数空间里找局部最优。

**正确顺序**：先验证量纲和量级（`|b| / 变量界` 的比值），再谈增益。

> **教训**：调参之前，先确认"参数的单位是什么"。`kp * 位置误差` 得到的是加速度还是力？如果单位不对，任何数值都没有意义。

#### 弯路 2：以为走圆弧能减少侧滑

**当时的想法**："原地转要侧滑，那走圆弧（滚动为主）应该就好多了。"

**为什么错**：侧滑 `v_y = ω·r_x` 只跟角速度和**前后偏置**有关，跟前进速度无关。实测侧滑分量在两种工况下**完全相同（±0.096 m/s）**。

**教训**：要证明"换个工况会更好"，先写出解析式。能写出解析式的，就别做实验。

#### 弯路 3：以为换速度执行器就行

**当时的想法**："力矩控制受摩擦限制，那用速度伺服（`<velocity>`）强制轮子转就行。"

**为什么错**：摩擦**在环内**。速度伺服的输出仍是力，还得通过同一个摩擦锥。实测 `kv=30`、力限提到 150 Nm，ω 仍然只有 0.028–0.06 rad/s。

**教训**：改变**控制信号的形式**不会改变**物理约束**。要绕过物理，必须改物理模型本身（几何/摩擦/自由度）。

#### 弯路 4：以为换求解器参数能救

试了 `noslip_iterations`、`impratio`、Newton、elliptic 四种，全部无效或更差。

**教训**：求解器参数调的是**数值精度**，不是**物理正确性**。如果现象有明确的物理量级解释（508 vs 689），别浪费时间在求解器上。

#### 弯路 5：相信了一个"通过"的验证脚本

旧版 `tools/validate.py` 打印"质量矩阵误差 2e-16"，看起来完美。实际上它比的模型里**根本没有底盘**（`fusestatic` 丢弃了 194 kg）。

**教训**：**任何验证脚本，第一步都要打印双方的关键规模量**（nv/nq/总质量/关节数）。规模对不上，后面的精度数字毫无意义。

> 这一条可以推广成一条通用规则：
> **验证脚本必须先证明"我在比的是我以为的东西"，再证明"它们相等"。**

---

## 8. 物理与运动学速查卡

### 8.1 刚体上任意一点的速度

```
v_p = v_O + ω × r
```

二维（绕 z 转 ω，质心速度 v_O=(v,0)）：

```
v_x = v − ω·r_y        ← 会随"左右偏置"变化
v_y =     ω·r_x        ← 会随"前后偏置"变化
```

### 8.2 车轮的能力边界

一个车轮只有 **1 个自由度**（绕轴自转）：
- **能吸收**：沿滚动方向的速度（通过自转 ⇒ 合法滚动）
- **不能吸收**：垂直于滚动方向的速度（⇒ 必须打滑）

自转贡献的方向：

```
ω_wheel ŷ × (−r_w ẑ) = −ω_wheel·r_w x̂     ← 只在 x 方向
```

⇒ **`v_y(接触点) ≡ v_y(轮心)`，与轮子怎么转无关。**

### 8.3 skid-steer（4 轮双轴）的宿命

```
                前后偏置 r_x = ±0.319
      ┌───────────────────────────────────┐
      │  RL ●                  ● FR       │
      │      ↙   (前轮向 +y 滑)  ↘        │
      │              ╳ (转向中心)          │
      │      ↖   (后轮向 −y 滑)  ↗        │
      │  FL ●                  ● RR       │
      └───────────────────────────────────┘
```

- 侧滑分量 `±ω·r_x` ⇒ 前轮和后轮被剪切
- 四个侧向摩擦形成一个**抵抗转向的纯力偶**
- 侧滑速度与 μ 无关（是运动学量），但**抵抗矩**与 μ 成正比

### 8.4 差速底盘的驱动能力

```
M_drive = 2·d_y·(F_right − F_left) = 2·d_y·(2·τ_diff / r_w)
```

本项目：`2 × 0.2755 × 2 × 35 / 0.0759 = 508 Nm`

### 8.5 侧滑阻力矩

```
M_resist = Σ |r_x,i| · μ · N_i = 4 · 0.319 · μ · (m·g/4)
```

| μ | M_resist | 与 M_drive(508) 比 |
|---|---|---|
| 0.5 | 345 Nm | 可以转 |
| 1.0 | 689 Nm | **转不动** |
| 1.5 | 1034 Nm | 更转不动 |

### 8.6 一个反直觉的推论

**μ 越大，原地转向越难**（因为侧滑阻力 ∝ μ）。这和"摩擦力大更好开"的直觉相反——因为原地转向需要的是**放弃抓地**，不是获得抓地。

这也是为什么实测 μ 扫描会出现"μ=1.5 时给 35 Nm 只转 0.05 rad/s，比 μ=0.9 更差"。

---

## 9. 代码片段库（可直接复用）

### 9.1 启动时的模型自检（防索引回归）

```cpp
bool validateModelLayout() {
    if (model_->dof_count != 12 || model_->qdot_size != 12 || model_->q_size != 13) {
        std::cerr << "unexpected: dof=" << model_->dof_count
                  << " nq=" << model_->q_size << " nv=" << model_->qdot_size << "\n";
        return false;
    }
    if (model_->mJoints[1].mJointType != RBDL::JointTypeTranslationXYZ) {
        std::cerr << "floating base root joint is not TranslationXYZ\n";
        return false;
    }
    // 打印关节表，让布局一目了然
    std::cout << "model layout:\n";
    for (unsigned int i = 1; i < model_->mJoints.size(); ++i) {
        const auto &j = model_->mJoints[i];
        if (j.mDoFCount == 0) continue;
        std::cout << "  body[" << i << "] '" << model_->GetBodyName(i)
                  << "' type=" << j.mJointType << " q_index=" << j.q_index
                  << " dof=" << j.mDoFCount << "\n";
    }
    return true;
}
```

预期输出：

```
model layout: dof=12 nq=13 nv=12
  body[1] '' type=11 q_index=0 dof=3          ← TranslationXYZ
  body[2] 'chassis_link' type=6 q_index=3 dof=3  ← Spherical
  body[3] 'ur_arm_shoulder_link' type=5 q_index=6 dof=1
  ...
```

### 9.2 FK 交叉校验（永久保留）

见 [2.3](#23-终极检查fk-交叉校验本次最有价值的一招)。

### 9.3 双模型对拍（`tools/validate.py` 骨架）

```python
import mujoco, numpy as np
WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]

def build_urdf_side(urdf):
    sp = mujoco.MjSpec.from_file(urdf)
    sp.compiler.fusestatic = False          # ← 关键
    sp.body("chassis_link").add_freejoint()
    for w in WHEELS:
        j = sp.joint(w)
        if j: sp.delete(j)
    return sp.compile()

def build_sim_side(scene):
    sp = mujoco.MjSpec.from_file(scene)
    for a in list(sp.actuators):
        if a.target in WHEELS: sp.delete(a)
    for w in WHEELS:
        j = sp.joint(w)
        if j: sp.delete(j)
    return sp.compile()

A, B = build_urdf_side("robot.urdf"), build_sim_side("scene.xml")

# 【第一步】先证明在比同一个东西
print("dims:", A.nv, B.nv, A.nq, B.nq)
print("mass:", A.body_subtreemass[0], B.body_subtreemass[0])
assert A.nv == B.nv and A.nq == B.nq, "维度不一致，后面的比较无意义！"

# 【第二步】再比精度
for yaw, q_arm in [(0, np.zeros(6)), (0.7, np.array([.3,-1.1,1.4,-.5,.9,.2]))]:
    for m, d in ((A, da), (B, db)):
        d.qpos[:] = 0; d.qpos[2] = 0.0257
        d.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]   # quat = w,x,y,z
        d.qpos[7:13] = q_arm
        mujoco.mj_forward(m, d)
    MA = np.zeros((A.nv,)*2); MB = np.zeros((B.nv,)*2)
    mujoco.mj_fullM(A, da, MA); mujoco.mj_fullM(B, db, MB)
    print("M rel err:", np.abs(MA-MB).max() / np.abs(MA).max())
    print("h err:", np.abs(da.qfrc_bias - db.qfrc_bias).max())
```

### 9.4 开环扫描（判断物理层是否是瓶颈）

```python
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path("scene.xml")
print("input -> steady output (linear? dead zone? cliff?)")
for u in np.arange(low, high, step):
    d = mujoco.MjData(m); d.qpos[2] = 0.026; mujoco.mj_forward(m, d)
    for _ in range(900): mujoco.mj_step(m, d)        # 落稳
    h = []
    for k in range(3000):                            # 6 s
        d.ctrl[0], d.ctrl[1], d.ctrl[2], d.ctrl[3] = -u, +u, -u, +u
        mujoco.mj_step(m, d)
        if k > 2100: h.append(d.qvel[5])             # 只取后 30%（稳态）
    print(f"  u={u:5.1f} -> out={np.mean(h):+7.3f} sd={np.std(h):6.3f}"
          f" drift={np.linalg.norm(d.qpos[:2]):.3f}")
```

**读法**：
- `sd/|mean| < 0.1` 且单调 ⇒ 线性区，可控
- `sd/|mean| > 0.5` ⇒ 极限环/打滑，不可控
- `mean ≈ 0` 但输入不小 ⇒ 死区

### 9.5 力矩预算核算（用纸算，别用实验）

```python
RW, DX, DY = 0.0759, 0.319, 0.2755     # 轮半径、前后偏置、左右偏置
N = 220.26 * 9.81 / 4                   # 单轮法向力

for mu in (0.5, 1.0):
    drive  = 2*DY * (2*35/RW)           # 35 Nm/轮打满
    resist = 4*DX * mu*N
    print(f"mu={mu}: drive={drive:.0f} Nm, resist={resist:.0f} Nm, "
          f"{'OK' if drive > resist else 'IMPOSSIBLE'}")
```

### 9.6 偏航优先的力矩分配

```cpp
const double limit   = std::min(tau_wheel_max, traction_torque());  // 电机 & 摩擦锥
const double yaw_cap = std::min(limit, tau_diff_max);
tau_yaw = std::clamp(tau_yaw, -yaw_cap, yaw_cap);

const double room_fwd = std::max(0.0, limit - std::abs(tau_yaw));
tau_fwd = std::clamp(tau_fwd, -room_fwd, room_fwd);

cmd.ctrl[FL] = tau_fwd - tau_yaw;
cmd.ctrl[FR] = tau_fwd + tau_yaw;
cmd.ctrl[RL] = tau_fwd - tau_yaw;
cmd.ctrl[RR] = tau_fwd + tau_yaw;
```

### 9.7 车体系底盘任务（非完整约束友好）

```cpp
// 差速底盘不能直接命令横向运动，所以把平面误差投影到航向角上
Eigen::MatrixXd A_base = Eigen::MatrixXd::Zero(2, nv);
A_base(0, 0) = std::cos(yaw);   // 车体前向 = TX*cos + TY*sin
A_base(0, 1) = std::sin(yaw);
A_base(1, 5) = 1.0;             // 偏航角加速度

const double e_fwd  = (target_pos - cur_pos).dot(Eigen::Vector2d(cos_yaw, sin_yaw));
const double v_fwd  = qdot[0]*cos_yaw + qdot[1]*sin_yaw;
b_base[0] = kp_x*e_fwd + kd_x*(v_des - v_fwd) + clamp(ki_x*I_x, ±int_limit);
b_base[1] = kp_yaw*e_yaw + kd_yaw*(w_des - w_z) + clamp(ki_yaw*I_yaw, ±int_limit);
```

**关键点**：
- `kp` 单位是 1/s²（因为是加速度任务）
- `w_des` 必须给前馈，否则斜坡参考永远滞后
- 积分项要**输出限幅式抗饱和**（误差反号就清零）

---

## 10. 症状 → 原因速查表

| 症状 | 最可能原因 | 一行验证 |
|---|---|---|
| 某个自由度完全不动 | 索引错 / 该行被硬约束卡死 | 打印 `A` 矩阵对应行 + 变量界 |
| 误差线性累积、速率只有参考的 60–90% | 缺速度前馈 + 积分饱和 | 检查 `b` 里有没有 `w_des` 项 |
| 有时动有时不动，日志刷 `max iterations` | QP 病态或失败后静默降级 | 打印每层 status + 失败率 |
| 只往一个方向冲、不转向 | 差动被 clamp 抹平 | 打印 `tau_fwd`, `tau_yaw`, 四轮指令 |
| 前馈不灵、需要巨大积分 | 模型 M/h 与仿真不一致 | 跑双模型对拍 |
| 臂能跟、底盘不跟 | 底盘任务优先级/预算被抢 | 交换两级优先级看是否改善 |
| 末端误差随底盘转角线性增长 | **QP 用执行器做不到的自由度作弊** | 打印 QP 解，看底盘横向加速度分量 |
| 增益越调越糟 | 量纲错（位置增益当加速度用） | 算 `max|b| / 变量上界` |
| 停在平台期不动、给力也不动 | 死区（静摩擦/间隙） | 开环扫描看是否有死区 |
| 目标一提高就跳变成失控 | 目标落在"陡坡"区 | 开环扫描看曲线斜率 |
| 换个轨迹形状没改善 | 瓶颈是运动学量（如 `ω·r_x`），与轨迹无关 | 写解析式，别做实验 |
| 换执行器类型没改善 | 物理约束在环内 | 做力矩预算核算 |
| 验证脚本"完美通过"但行为不对 | 验证对象错了 | 打印双方 nv/nq/总质量 |

---

## 11. 附录：本次全部实测数据

### A. 硬件/软件版本

- MuJoCo 3.7.0（仿真库 + Python bindings 3.17.0）
- RBDL 3.3.1
- OSQP 1.x（新 API：`osqp_setup` / `OSQPSolver` / `OSQPCscMatrix_set_data`）
- 机器人：Ridgeback 底盘 + UR5，轮子 x=±0.319 m、y=±0.2755 m、r=0.0759 m
- 总质量 220.26 kg，绕世界 z 的 Izz = 15.64 kg·m²

### B. 修复前后对比

| 指标 | 修复前 | 修复后 |
|---|---|---|
| BasePos 误差（直线 v=0.15 m/s） | 180 → 560 mm（发散） | **2–6 mm** |
| BasePos 误差（原地转 0.25 rad/s） | — | 390–450 mm（打滑漂移，已接受） |
| EE 误差 | 6–40 mm | **2–9 mm** |
| QP 失败率 | ≥5500 次 / 21 s | **1–2 次 / 16 s** |
| 底盘 yaw 可达速率 | ~0.03 rad/s | **0.25 rad/s** |
| FK 交叉校验 gap | 未做 | **0.00 mm** |
| 模型一致性 M(q) | 轮子轴错误 | **4.5e-4 相对误差** |

### C. 速率跟随（最终配置，μ=0.5）

| 参考 ω | 实测 ω | 跟随率 |
|---|---|---|
| 0.15 rad/s | 0.1375 | 92% |
| 0.25 rad/s | 0.2225 | 89% |
| 0.30 rad/s | 0.2540 | 85%（落在陡坡，不可达） |

### D. 摩擦对可达速率的影响

| μ_eff | 稳定可达 ω 上限 | 断点力矩 |
|---|---|---|
| 1.0 | 0.03 rad/s | ~26 Nm |
| 0.5 | 0.25 rad/s | ~14 Nm |

### E. 已排除的"疑似解药"

见 [6.5](#65-常见看起来能救命但没用本次全部实测排除了)。

---

## 附：怎么把这份文档当"提示词"用

如果你下次想让 AI 帮忙，可以这样开场，能省掉大量来回：

> 我在排查一个移动操作机器人的跟踪问题：
> - 现象：<具体指标>，<哪些跟得上、哪些跟不上>
> - 已经排除了：<按本文五层，哪几层验证过>
> - 当前卡在：<第几层，具体现象>
> - 请先帮我判断这是五层里的哪一层，再给实验方案，不要直接给调参建议。

同时把本文的 **第 1 节（五层分解）** 和 **第 10 节（症状速查表）** 贴给它，它就不会一上来让你调 PID。

---

*本文档基于 `ridgeback_ur5` + `dynamic/wbc` 项目的真实排查过程整理。所有数字均可在 `tools/validate.py`、以及文中给出的扫描脚本中复现。*
