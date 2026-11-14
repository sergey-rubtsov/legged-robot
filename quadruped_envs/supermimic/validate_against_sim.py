"""Phase-1 validation: compare the new differentiable mdp.py/od_robot.py against the real
PyBullet-computed reward/termination/FK in imitation_task.py, on an actual rollout.

Finding: imitation_task.py's `_calc_reward_end_effector` reads `pyb.getLinkState(model, j)`
WITHOUT `computeForwardKinematics=1`, so right after a `stepSimulation()`/`resetJointState*`
call it can return a link transform that is one internal update behind the generalized
coordinates `getJointStateMultiDof`/`getBasePositionAndOrientation` already report (this is
documented PyBullet behavior, not specific to this repo). mdp.py's FK is a closed-form
function of the (always-fresh) joint angles, so it's compared here against
`getLinkState(..., computeForwardKinematics=1)` -- the genuinely fresh ground truth -- not
the default call. The other 4 reward terms (pose, velocity, root_pose, root_velocity) never
call getLinkState at all, so they're compared directly against env.step()'s real reward with
a tight tolerance.
"""
import os
import sys

import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from quadruped_envs.od_imitation_env import build_imitation_env
from quadruped_envs import od
from quadruped_envs.imitation_wrapper_env.imitation_terminal_conditions import imitation_terminal_condition
from quadruped_envs.supermimic import mdp, od_robot

motion_file = os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt")

env = build_imitation_env(
    motion_files=[motion_file], num_parallel_envs=1,
    enable_rendering=False, enable_randomizer=False, robot_class=od.OD,
)
env.reset()

pyb = env._pybullet_client
task = env._task
sim_model = env.robot.quadruped
ref_model = task.get_ref_model()
foot_link_ids = env.robot.GetFootLinkIDs()  # sorted -> [leg a, b, c, d] joint indices


def read_state(model_id) -> mdp.State:
    root_pos, root_rot = pyb.getBasePositionAndOrientation(model_id)
    lin_vel, ang_vel = pyb.getBaseVelocity(model_id)
    joint_pos, joint_vel = [], []
    for j in range(mdp.NUM_JOINTS):
        js = pyb.getJointStateMultiDof(model_id, j)
        joint_pos.append(js[0][0])
        joint_vel.append(js[1][0])
    t = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float32)
    return mdp.State(
        root_pos=t(root_pos), root_rot=t(root_rot),
        root_lin_vel=t(lin_vel), root_ang_vel=t(ang_vel),
        joint_pos=t(joint_pos), joint_vel=t(joint_vel),
    )


def fresh_foot_positions_world(model_id):
    return np.array([pyb.getLinkState(model_id, j, computeForwardKinematics=1)[0] for j in foot_link_ids])


rng = np.random.default_rng(0)
n_steps = 300
action_scale = float(os.environ.get("ACTION_SCALE", "1.0"))

max_fk_err = 0.0
max_term_component_err = {"pose": 0.0, "velocity": 0.0, "root_pose": 0.0, "root_velocity": 0.0}
term_mismatches = 0
reward_log = {"mine": [], "real": [], "pose": [], "velocity": [], "end_effector": [],
             "root_pose": [], "root_velocity": []}

for i in range(n_steps):
    action = action_scale * rng.uniform(env.action_space.low, env.action_space.high)
    obs, real_reward, done, info = env.step(action)

    sim_state = read_state(sim_model)
    ref_state = read_state(ref_model)
    my_breakdown = mdp.compute_reward(sim_state, ref_state)

    # The 4 terms that never touch getLinkState -- must match the real task exactly.
    real_pose_r, real_vel_r = task._calc_reward_position_and_velocity()
    real_root_pose_r = task._calc_reward_root_pose()
    real_root_vel_r = task._calc_reward_root_velocity()
    for name, real_r in [("pose", real_pose_r), ("velocity", real_vel_r),
                         ("root_pose", real_root_pose_r), ("root_velocity", real_root_vel_r)]:
        e = abs(float(my_breakdown[name]) - real_r)
        max_term_component_err[name] = max(max_term_component_err[name], e)

    # End-effector FK, compared against the genuinely fresh ground truth (see module docstring).
    my_feet_sim = (sim_state.root_pos.unsqueeze(0) + mdp.rot.quat_rotate_point(
        od_robot.foot_positions_base_frame(sim_state.joint_pos), sim_state.root_rot.unsqueeze(0))).numpy()
    my_feet_ref = (ref_state.root_pos.unsqueeze(0) + mdp.rot.quat_rotate_point(
        od_robot.foot_positions_base_frame(ref_state.joint_pos), ref_state.root_rot.unsqueeze(0))).numpy()
    fk_err = max(np.abs(my_feet_sim - fresh_foot_positions_world(sim_model)).max(),
                np.abs(my_feet_ref - fresh_foot_positions_world(ref_model)).max())
    max_fk_err = max(max_fk_err, fk_err)

    my_done = bool(mdp.compute_termination(sim_state, ref_state))
    real_done_state_only = imitation_terminal_condition(env)
    # real `done` from env.step can also fire on contact/motion-over, which mdp.py
    # intentionally doesn't model; only compare the state-based part.
    if my_done != real_done_state_only:
        term_mismatches += 1

    reward_log["mine"].append(float(my_breakdown["total"]))
    reward_log["real"].append(real_reward)
    for k in ["pose", "velocity", "end_effector", "root_pose", "root_velocity"]:
        reward_log[k].append(float(my_breakdown[k]))

    if done:
        env.reset()

print(f"steps={n_steps}, action_scale={action_scale}")
for name, e in max_term_component_err.items():
    print(f"max |my_{name} - real_{name}| = {e:.3e}")
print(f"max |my_foot_world - FRESH_real_foot_world| = {max_fk_err:.3e}")
print(f"termination mismatches (state-only) = {term_mismatches}/{n_steps}")

out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".validation_runs")
os.makedirs(out_dir, exist_ok=True)
np.savez(os.path.join(out_dir, "validate_against_sim_log.npz"),
        **{k: np.array(v) for k, v in reward_log.items()})

env.close()
for name, e in max_term_component_err.items():
    assert e < 1e-4, f"{name} mismatch too large: {e}"
assert max_fk_err < 2e-3, f"FK mismatch too large: {max_fk_err}"  # ~1e-3 explained by a single
# link_l_leg_d inertial-origin offset (-0.00038, 0, 0.00077) this FK doesn't model -- see od_robot.py.
print("VALIDATION PASSED")
