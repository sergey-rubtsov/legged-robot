"""Instrumented rollout diagnostic for a trained policy: checks for world constraints, foot/
ground friction coefficients, segments the rollout by episode resets (so reset-induced position
teleports aren't mistaken for either stillness or locomotion), and measures per-foot lateral
slip during single-foot stance windows. Written to answer a very concrete question: "the robot
looks pinned in the rendered clip and its feet look like they're sliding -- is something
constraining it, and what's going on with friction?" (answer, as of the pace_run1 checkpoint:
no constraints, friction is fine (ground 0.9, feet 0.5), but the render's chase camera hides
real translation, the episode actually falls and resets every few seconds, and feet do slip
measurably (0.04-0.15 m in under 0.1 s) during nominal stance -- consistent with
imag_survival's flat ~0.013 during training, now confirmed qualitatively too).

Usage: python -m quadruped_envs.supermimic.diagnose_rollout --ckpt path/to/last.pt
"""
import argparse
import os
import sys

import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", type=str, required=True)
    p.add_argument("--motion_file", type=str,
                   default=os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt"))
    p.add_argument("--steps", type=int, default=400)
    p.add_argument("--seed", type=int, default=None,
                   help="fixed seed for env.reset()/torch/numpy so displacement numbers are "
                        "reproducible, not an anecdote from whichever un-seeded resets occurred "
                        "(see feedback_verify_visual_claims memory)")
    p.add_argument("--recovery_init_prob", type=float, default=0.0,
                   help="probability of starting each episode from the scripted fallen-on-its-"
                        "back pose instead of a normal reference-tracked reset -- only meaningful "
                        "when --motion_file is the recovery motion (e.g. recovery.txt). Also "
                        "switches the per-step report from locomotion displacement to orientation "
                        "(up_dot) tracking, since 'does it get up' isn't a displacement question.")
    args = p.parse_args()

    if args.seed is not None:
        torch.manual_seed(args.seed)
        np.random.seed(args.seed)

    from quadruped_envs.od_imitation_env import build_imitation_env
    from quadruped_envs import od
    from quadruped_envs.supermimic import od_robot, rotations as rot
    from quadruped_envs.supermimic.mdp import NUM_JOINTS, OBS_DIM, XI_DIM, State, mat_state_from, obs as mdp_obs
    from quadruped_envs.supermimic.motion import CONTROL_DT, BakedMotion
    from quadruped_envs.supermimic.networks import Policy

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    ck_args = ck["args"]
    K, Hh = ck_args["goal_frames"], ck_args["hist"]
    # Must match whatever --pi_ref_pd* the checkpoint was actually trained with -- see
    # demo_live.py's identical note.
    ref_pd = None
    if ck_args.get("pi_ref_pd"):
        from quadruped_envs.imitation_wrapper_env.od_motor import TORQUE_MULTIPLIER
        ref_pd = (ck_args["pi_ref_pd_kp"], ck_args["pi_ref_pd_kd"], float(TORQUE_MULTIPLIER), ck_args["pi_ref_pd_clip"])
    pi = Policy(OBS_DIM, 39 * K, hist=Hh, ref_pd=ref_pd)
    pi.load_state_dict(ck["pi"])
    pi.eval()

    # Must match whatever --control_mode the checkpoint was actually trained with -- see
    # demo_live.py's identical note (a position-trained policy's actions are joint-position-target
    # offsets, not raw torques).
    is_position_mode = ck_args.get("control_mode") == "position"
    motor_control_mode = None
    default_q_np = None
    if is_position_mode:
        from motion_imitation.robots import robot_config
        from quadruped_envs.supermimic.mdp import (ACTION_POS_SCALE, DEFAULT_Q,
                                                    RECOVERY_Q_CENTER, RECOVERY_Q_HALF_RANGE)
        motor_control_mode = robot_config.MotorControlMode.POSITION
        default_q_np = DEFAULT_Q.numpy().copy()
        recovery_center_np = RECOVERY_Q_CENTER.numpy().copy()
        recovery_half_range_np = RECOVERY_Q_HALF_RANGE.numpy().copy()

    motion = BakedMotion(args.motion_file)
    recovery_motion_names = {os.path.basename(args.motion_file)} if args.recovery_init_prob > 0 else frozenset()
    env = build_imitation_env(motion_files=[args.motion_file], num_parallel_envs=1,
                              enable_rendering=False, enable_randomizer=False, robot_class=od.OD,
                              recovery_motion_names=recovery_motion_names,
                              recovery_init_prob=args.recovery_init_prob,
                              motor_control_mode=motor_control_mode)
    if args.seed is not None:
        env.seed(args.seed)
    env.reset()
    task = env._task
    pyb = env._pybullet_client
    sim_model = env.robot.quadruped
    ground = env.get_ground()
    foot_link_ids = env.robot.GetFootLinkIDs()

    print("=== constraints ===")
    print("num constraints in world:", pyb.getNumConstraints())
    print("\n=== dynamics info (friction) ===")
    print("ground (plane) lateral friction:", pyb.getDynamicsInfo(ground, -1)[1])
    for j in foot_link_ids:
        info = pyb.getDynamicsInfo(sim_model, j)
        print(f"  foot link {j}: lateralFriction={info[1]}, mass={info[0]}")

    obs_hist = torch.zeros(1, Hh, OBS_DIM)
    a_prev = torch.zeros(1, NUM_JOINTS)
    # Nominal sim-context -- see demo_live.py's identical note (this script also drives a single,
    # non-randomized PyBullet instance, not one of TrackingSim's workers).
    xi = torch.zeros(1, XI_DIM)
    xi[:, 0] = 0.85
    log = []

    for i in range(args.steps):
        t = torch.tensor([task._get_motion_time()], dtype=torch.float32)
        ref = motion.frame_state(t)

        root_pos, root_rot = pyb.getBasePositionAndOrientation(sim_model)
        lin_vel, ang_vel = pyb.getBaseVelocity(sim_model)
        joint_pos = np.array([pyb.getJointStateMultiDof(sim_model, j)[0][0] for j in range(NUM_JOINTS)], dtype=np.float32)
        joint_vel = np.array([pyb.getJointStateMultiDof(sim_model, j)[1][0] for j in range(NUM_JOINTS)], dtype=np.float32)
        sim_state = State(
            root_pos=torch.tensor(root_pos, dtype=torch.float32).unsqueeze(0),
            root_rot=torch.tensor(root_rot, dtype=torch.float32).unsqueeze(0),
            root_lin_vel=torch.tensor(lin_vel, dtype=torch.float32).unsqueeze(0),
            root_ang_vel=torch.tensor(ang_vel, dtype=torch.float32).unsqueeze(0),
            joint_pos=torch.tensor(joint_pos).unsqueeze(0),
            joint_vel=torch.tensor(joint_vel).unsqueeze(0),
        )
        S_mat = mat_state_from(sim_state)

        feet_base = od_robot.foot_positions_base_frame(torch.tensor(joint_pos).unsqueeze(0))
        root_pos_t = torch.tensor(root_pos, dtype=torch.float32).view(1, 1, 3)
        root_rot_t = torch.tensor(root_rot, dtype=torch.float32).view(1, 1, 4)
        feet_world = root_pos_t + rot.quat_rotate_point(feet_base, root_rot_t)

        o = mdp_obs(S_mat, a_prev, ref.R, xi)
        obs_hist = torch.cat((obs_hist[:, 1:], o.unsqueeze(1)), 1)
        g = motion.goal(t, K, CONTROL_DT)

        with torch.no_grad():
            action = torch.clamp(pi(obs_hist, g), -1.0, 1.0)

        action_np = action.squeeze(0).numpy()
        if is_position_mode and task._is_recovery_motion():
            sim_action = recovery_center_np + action_np * recovery_half_range_np
        elif is_position_mode:
            sim_action = default_q_np + action_np * ACTION_POS_SCALE
        else:
            sim_action = action_np
        _, reward, done, info = env.step(sim_action)
        a_prev = action

        contacts = pyb.getContactPoints(bodyA=sim_model, bodyB=ground)
        contact_links = sorted(set(c[3] for c in contacts))
        # up_dot: z-component of the body's own local z-axis in world frame -- same quantity
        # mdp.py's recovery_progress_reward shapes toward (1 = upright, -1 = upside down), the
        # natural "did it get up" measure for the recovery task (unlike locomotion displacement,
        # which doesn't mean anything for a task that's supposed to end up standing in place).
        up_dot = float(rot.quat_to_mat(root_rot_t.squeeze(1))[0, 2, 2])
        # up_dot alone is a trap for recovery: it only measures body ORIENTATION, and a sharp
        # flail/bounce off the wide-range joints can tip the body through near-vertical for a
        # handful of steps without anything resembling the actual standing reference -- confirmed
        # in practice (2026-11-11): a "peak up_dot=0.33" moment had root_h=0.19m (real standing
        # is 0.29-0.33m, see idle.txt) and mean per-joint error ~2.6 rad (~150 degrees) vs the
        # reference, i.e. a momentary tumble-through, not a stand. Track both so later reporting
        # can tell the difference instead of celebrating an orientation spike alone.
        root_h = float(root_pos[2])
        joint_err = float(np.abs(joint_pos - ref.q.squeeze(0).numpy()).mean())
        log.append({"i": i, "pos": np.array(root_pos), "done": bool(done), "up_dot": up_dot,
                   "root_h": root_h, "joint_err": joint_err,
                   "contacts": contact_links, "feet_world": feet_world.squeeze(0).numpy().copy()})

        if done:
            env.reset()
            obs_hist.zero_()
            a_prev.zero_()

    env.close()

    segments, seg_start = [], 0
    for e in log:
        if e["done"]:
            segments.append((seg_start, e["i"]))
            seg_start = e["i"] + 1
    if seg_start < len(log):
        segments.append((seg_start, len(log) - 1))

    print(f"\n=== episode segments (reset = fell or drifted too far) ===")
    print(f"total steps: {len(log)}, number of segments: {len(segments)}")
    for s, e in segments:
        p0, p1 = log[s]["pos"], log[e]["pos"]
        length = e - s + 1
        dxy = (p1 - p0)[:2]
        dist = np.linalg.norm(dxy)
        avg_speed = dist / (length * CONTROL_DT) if length > 0 else 0.0
        up0, up1, up_max = log[s]["up_dot"], log[e]["up_dot"], max(x["up_dot"] for x in log[s:e+1])
        extra = f"  up_dot {up0:+.2f}->{up1:+.2f} (max {up_max:+.2f})" if args.recovery_init_prob > 0 else ""
        print(f"  steps [{s},{e}] (len {length}): dxy=({dxy[0]:+.4f},{dxy[1]:+.4f}) net_xy_disp={dist:.4f} m -> {avg_speed:.4f} m/s avg{extra}")

    if args.recovery_init_prob > 0:
        # up_dot ALONE is a trap: it's pure body orientation, and a sharp flail/bounce off the
        # wide-range recovery joints can tip the body through near-vertical for a handful of
        # steps with nothing resembling the actual standing reference underneath it (confirmed in
        # practice, 2026-11-11: a peak up_dot=0.33 moment had root_h=0.19m -- real standing is
        # 0.29-0.33m, see idle.txt -- and mean per-joint error ~2.6 rad/~150 degrees vs the
        # reference). STAND_HEIGHT_MIN/STAND_JOINT_ERR_MAX below are deliberately generous (well
        # short of the reference's own 0.29-0.33m / an exact pose match) -- the point is
        # distinguishing "momentary tumble-through" from "something recognizably stand-like", not
        # demanding a perfect stand.
        STAND_HEIGHT_MIN = 0.25
        STAND_JOINT_ERR_MAX = 1.0
        print("\n=== recovery-specific summary (up_dot: 1=upright, -1=upside down) ===")
        fallen_segments = [(s, e) for s, e in segments if log[s]["up_dot"] < 0]
        oriented = [(s, e) for s, e in fallen_segments if max(x["up_dot"] for x in log[s:e+1]) > 0.7]
        standing = [(s, e) for s, e in fallen_segments
                   if any(x["up_dot"] > 0.7 and x["root_h"] > STAND_HEIGHT_MIN
                          and x["joint_err"] < STAND_JOINT_ERR_MAX for x in log[s:e+1])]  # fmt: skip
        print(f"segments starting fallen (up_dot<0): {len(fallen_segments)}/{len(segments)}")
        print(f"of those, oriented upright (up_dot>0.7) at some point: {len(oriented)}/{len(fallen_segments) or 1}")
        print(f"of those, ALSO stand-like at that moment (root_h>{STAND_HEIGHT_MIN}m AND "
              f"joint_err<{STAND_JOINT_ERR_MAX}rad): {len(standing)}/{len(fallen_segments) or 1}")
        for s, e in fallen_segments:
            ups = [x["up_dot"] for x in log[s:e+1]]
            peak_idx = int(np.argmax(ups))
            peak = log[s:e+1][peak_idx]
            print(f"  steps [{s},{e}] (len {e-s+1}): start={ups[0]:+.2f} end={ups[-1]:+.2f} "
                 f"max={max(ups):+.2f} min={min(ups):+.2f}  @peak: root_h={peak['root_h']:.3f}m "
                 f"joint_err={peak['joint_err']:.3f}rad")  # fmt: skip

    # Per-segment net_xy_disp is a MAGNITUDE (always >=0), and each segment's reference origin is
    # re-synced to the robot's own position/heading at reset time (see imitation_task.py's
    # _reset_ref_motion), so segments can displace in essentially unrelated directions after a
    # fall reorients the robot -- real per-segment movement does NOT imply consistent net travel
    # in one world-frame direction. Report the latter explicitly too (see
    # feedback_verify_visual_claims memory: a fixed-seed world-frame net displacement over the
    # WHOLE rollout, not per-segment magnitudes, is what answers "does it actually go somewhere").
    p_first, p_last = log[0]["pos"], log[-1]["pos"]
    total_dxy = (p_last - p_first)[:2]
    total_net = np.linalg.norm(total_dxy)
    # Reference motions in this project's data files all move in +x (see motion file dx check),
    # so a negative dx here (even with a large net_disp magnitude) means the policy is moving
    # opposite to the reference's own intended travel direction, not just "slowly"/"not yet".
    print(f"\n=== whole-rollout net displacement (fixed start->end, {len(log)} steps, ")
    print(f"    ignores segment resets -- the real 'does it go somewhere' answer) ===")
    print(f"    start_xy={p_first[:2]}, end_xy={p_last[:2]}, dxy=({total_dxy[0]:+.4f},{total_dxy[1]:+.4f}), "
          f"net_disp={total_net:.4f} m ({total_net / (len(log) * CONTROL_DT):.4f} m/s avg over whole rollout)")

    print("\n=== single-foot contact stretches (potential stance phases): xy slip ===")
    run_link, run_start = None, None
    for idx, e in enumerate(log):
        cl = e["contacts"]
        link = cl[0] if len(cl) == 1 else None
        if link is not None and link not in foot_link_ids:
            # Single contact, but not a foot -- e.g. the base/body link (index -1) touching the
            # ground, the normal contact during a recovery episode's fallen-on-back pose. Not a
            # foot stance, so don't treat it as one (foot_link_ids.index(link) below would raise
            # ValueError on it otherwise).
            link = None
        if link != run_link:
            if run_link is not None and idx - run_start >= 8:
                foot_idx = foot_link_ids.index(run_link)
                p_start = log[run_start]["feet_world"][foot_idx]
                p_end = log[idx - 1]["feet_world"][foot_idx]
                slip = np.linalg.norm((p_end - p_start)[:2])
                print(f"  steps [{run_start},{idx - 1}] foot link {run_link}: slip = {slip:.4f} m over {idx - run_start} steps")
            run_link, run_start = link, idx


if __name__ == "__main__":
    main()
