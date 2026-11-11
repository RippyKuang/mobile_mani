#!/usr/bin/env python3
"""Validate that the RBDL model (built from the URDF) matches the MuJoCo model (MJCF).

Both models are reduced to the same 12-DoF representation -- floating base (6) +
arm (6), with the four wheel joints welded -- and compiled by MuJoCo, so the
comparison is apples-to-apples:

  * RBDL-equivalent : URDF + free joint on chassis_link, wheel joints deleted
  * Sim-equivalent  : MJCF with the wheel joints and wheel motors deleted

Checks: total mass, center of mass, joint-space mass matrix M(q), gravity/bias
force vector h(q), and the end-effector pose + Jacobian at several arm poses.

Run from the repository root:

    python tools/validate.py
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

URDF = "ridgeback_ur5/urdf/ridgeback_ur5.urdf"
SCENE = "ridgeback_ur5/mjcf/scene.xml"
WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]
EE_BODY = "ur_arm_wrist_3_link"

BASE_Z = 0.0257  # chassis_link height once the wheels rest on the floor

TOL_MASS = 1e-3   # kg   (total mass)
TOL_COM = 1e-3    # m    (center of mass)
TOL_M_REL = 1e-2  # -    (relative error of M(q))
TOL_H = 1.0       # N/Nm (bias force vector)
TOL_EE = 2e-3     # m    (end-effector position)
TOL_JAC = 1e-2    # -    (relative error of the EE Jacobian)

failures = []


def report(name, value, tol, unit=""):
    ok = value <= tol
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<42s} {value:>12.3e} {unit:<3s} (tol {tol:.1e})")
    if not ok:
        failures.append(name)
    return ok


def build_rbdl_equivalent(urdf_path):
    """What RBDL sees: floating base + arm; the welded links keep their inertia."""
    spec = mujoco.MjSpec.from_file(urdf_path)
    # fusestatic=true (the URDF default) fuses the joint-less root link into the
    # world body and DISCARDS its inertia -- that is what made the old script lie.
    spec.compiler.fusestatic = False
    spec.body("chassis_link").add_freejoint()
    for w in WHEELS:
        # the URDF importer may already have merged the welded joints away
        joint = spec.joint(w)
        if joint is not None:
            spec.delete(joint)
    return spec.compile()


def build_sim_equivalent(scene_path):
    """What MuJoCo simulates, reduced to the same 12 DoF."""
    spec = mujoco.MjSpec.from_file(scene_path)
    for act in list(spec.actuators):
        if act.target in WHEELS:
            spec.delete(act)
    for w in WHEELS:
        joint = spec.joint(w)
        if joint is not None:
            spec.delete(joint)
    return spec.compile()


def check_inertial_coverage(urdf_path):
    """RBDL only reads <inertial>; MuJoCo also infers mass from <collision> geoms."""
    root = ET.parse(urdf_path).getroot()
    suspicious = []
    for link in root.findall("link"):
        if link.find("inertial") is not None:
            continue
        if link.find("collision") is not None:
            # RBDL: massless. MuJoCo: mass inferred from the collision mesh.
            suspicious.append(link.get("name"))
    print("\n[inertial coverage] links without <inertial> that have collision geometry:")
    if suspicious:
        for name in suspicious:
            print(f"  FAIL  '{name}' has collision geometry but no <inertial> "
                  f"-> RBDL sees zero mass, MuJoCo does not")
        failures.extend(suspicious)
    else:
        print("  PASS  none (RBDL and MuJoCo agree on every link's mass)")
    return suspicious


def total_com(model, data):
    mass = 0.0
    com = np.zeros(3)
    for b in range(1, model.nbody):
        m = model.body_mass[b]
        if m <= 0.0:
            continue
        mass += m
        com += m * data.xipos[b]
    return mass, com / mass


def set_state(model, data, yaw, q_arm):
    data.qpos[:] = 0.0
    data.qpos[0:3] = [0.0, 0.0, BASE_Z]
    data.qpos[3:7] = [np.cos(0.5 * yaw), 0.0, 0.0, np.sin(0.5 * yaw)]  # quat = w,x,y,z
    data.qpos[7:13] = q_arm
    mujoco.mj_forward(model, data)


def main():
    urdf_model = build_rbdl_equivalent(URDF)
    sim_model = build_sim_equivalent(SCENE)

    print("=" * 78)
    print("model summary")
    print("=" * 78)
    for label, m in (("RBDL-equivalent", urdf_model), ("sim-equivalent ", sim_model)):
        joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
        print(f"  {label}: nv={m.nv} nq={m.nq} njnt={m.njnt}")
        print(f"      joints: {joints}")

    if urdf_model.nv != 12 or sim_model.nv != 12:
        print("\n  FAIL  expected nv=12 (floating base + 6 arm joints) on both sides")
        failures.append("dof count")
        return summarize()
    if urdf_model.nv != sim_model.nv or urdf_model.nq != sim_model.nq:
        print("\n  FAIL  the two models do not even have the same dimensions")
        failures.append("dimensions")
        return summarize()

    check_inertial_coverage(URDF)

    urdf_data = mujoco.MjData(urdf_model)
    sim_data = mujoco.MjData(sim_model)

    # --- static properties -------------------------------------------------
    print("\n[static model]")
    # put both models in the same reference state first (xipos is only valid after
    # mj_forward, otherwise the COM check compares 0 against 0)
    set_state(urdf_model, urdf_data, 0.0, np.zeros(6))
    set_state(sim_model, sim_data, 0.0, np.zeros(6))
    mass_u, com_u = total_com(urdf_model, urdf_data)
    mass_s, com_s = total_com(sim_model, sim_data)
    print(f"  total mass: RBDL {mass_u:.6f} kg / sim {mass_s:.6f} kg")
    print(f"  center of mass: RBDL {np.round(com_u, 5)} / sim {np.round(com_s, 5)}")
    report("total mass difference", abs(mass_u - mass_s), TOL_MASS, "kg")
    report("center of mass difference", float(np.max(np.abs(com_u - com_s))), TOL_COM, "m")

    # --- dynamic properties over several poses ----------------------------
    ee_u = mujoco.mj_name2id(urdf_model, mujoco.mjtObj.mjOBJ_BODY, EE_BODY)
    ee_s = mujoco.mj_name2id(sim_model, mujoco.mjtObj.mjOBJ_BODY, EE_BODY)

    poses = [
        (0.0, np.zeros(6)),
        (0.7, np.array([0.3, -1.1, 1.4, -0.5, 0.9, 0.2])),
        (-2.1, np.array([-1.2, -0.4, 0.8, 1.6, -0.7, 2.4])),
    ]
    M_u = np.zeros((urdf_model.nv, urdf_model.nv))
    M_s = np.zeros((sim_model.nv, sim_model.nv))
    jacp_u = np.zeros((3, urdf_model.nv))
    jacr_u = np.zeros((3, urdf_model.nv))
    jacp_s = np.zeros((3, sim_model.nv))
    jacr_s = np.zeros((3, sim_model.nv))

    print("\n[dynamic model]  (M(q) and h(q) at rest, plus the end-effector)")
    worst_m_rel = 0.0
    worst_h = 0.0
    worst_ee = 0.0
    worst_jac = 0.0

    for yaw, q_arm in poses:
        set_state(urdf_model, urdf_data, yaw, q_arm)
        set_state(sim_model, sim_data, yaw, q_arm)

        mujoco.mj_fullM(urdf_model, urdf_data, M_u)
        mujoco.mj_fullM(sim_model, sim_data, M_s)
        scale = float(np.max(np.abs(M_u)))
        worst_m_rel = max(worst_m_rel, float(np.max(np.abs(M_u - M_s))) / scale)

        worst_h = max(worst_h, float(np.max(np.abs(urdf_data.qfrc_bias - sim_data.qfrc_bias))))

        worst_ee = max(worst_ee, float(np.linalg.norm(urdf_data.xpos[ee_u] - sim_data.xpos[ee_s])))

        mujoco.mj_jacBody(urdf_model, urdf_data, jacp_u, jacr_u, ee_u)
        mujoco.mj_jacBody(sim_model, sim_data, jacp_s, jacr_s, ee_s)
        jac_u = np.vstack((jacp_u, jacr_u))
        jac_s = np.vstack((jacp_s, jacr_s))
        jworst = float(np.max(np.abs(jac_u - jac_s))) / max(1e-9, float(np.max(np.abs(jac_u))))
        worst_jac = max(worst_jac, jworst)

    print(f"  worst end-effector position gap: {(worst_ee * 1000.0):.3f} mm")
    report("mass matrix M(q) rel. error", worst_m_rel, TOL_M_REL)
    report("bias force h(q) absolute error", worst_h, TOL_H, "N")
    report("end-effector position error", worst_ee, TOL_EE, "m")
    report("end-effector Jacobian rel. error", worst_jac, TOL_JAC)

    return summarize()


def summarize():
    print("\n" + "=" * 78)
    if failures:
        print(f"RESULT: FAIL ({len(failures)} check(s) failed): {', '.join(failures)}")
        return 1
    print("RESULT: PASS -- the RBDL model and the MuJoCo model agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
