"""Play a trained checkpoint back live in a PyBullet GUI window, for watching on-screen rather
than rendering to a GIF (see eval_policy_rollout.py for the GIF-capture version, used when a
shareable/persistent artifact is more appropriate than a live window). Deterministic (no
exploration noise). Paced to roughly real-time with time.sleep -- PyBullet's GUI mode doesn't
auto-pace a loop driven by explicit stepSimulation() calls.

NOTE: avoid per-frame addUserDebugLine/addUserDebugText or similar debug-draw calls while this
window is open -- see the pybullet_macos_debug_draw_bug project memory (desyncs the GUI's
shared-memory connection on this Mac's Metal backend after ~60-70 frames). The on-screen controls
text below is added ONCE at startup, and the "current gait" status text is only ever replaced (via
replaceItemUniqueId, not a fresh addUserDebugText call) on an actual key-press event -- never every
frame -- to stay well clear of that bug.

Usage: python -m quadruped_envs.supermimic.demo_live --ckpt path/to/last.pt \
         --motion_file quadruped_envs/data/motions/od/gallop.txt

--motion_file accepts a comma-separated list (e.g. "pace.txt,trot.txt,gallop.txt") -- with more
than one entry, press N/P in the GUI window to switch forward/backward through them LIVE: the
robot keeps moving exactly as it was (no teleport, no reset) and simply starts being judged
against the new gait from the next step on, so it has to pick up the new reference on the fly.
Press R for a REAL reset instead -- with --recovery_motion also set to one of the listed files,
it forces a teleport to a scripted fallen-on-its-back start for THAT motion specifically (same
one-shot mechanic as --recovery_init_prob 1.0, just triggered on demand); without
--recovery_motion, R is just a plain reset of whichever gait is currently selected.

Pass --no_reset to run indefinitely without ever resetting on a fall/drift-too-far/episode-length
"done" -- confirmed there's no separate internal auto-reset in env.step() to fight (the only
place a reset is ever triggered is this script's own `if done: env.reset()`, so skipping that
call is sufficient), so the robot just keeps simulating forward from wherever it ends up, letting
you watch contact/impact behavior continuously instead of every fall snapping the view back to a
reference-synced start pose. Combine with --steps 0 to run until the window is closed.
"""
import argparse
import os
import sys
import time

import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", type=str, required=True, help="path to a train.py checkpoint (last.pt or ckpt_NNNNNN.pt)")
    p.add_argument("--motion_file", type=str,
                   default=os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt"),
                   help="comma-separated list to enable live N/P gait switching in the GUI "
                        "(single file for the old one-gait behavior, unchanged)")
    p.add_argument("--steps", type=int, default=2000, help="real steps to play before exiting (0 = until window closed)")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--no_reset", action="store_true",
                   help="never call env.reset() on a fall/drift-too-far 'done' -- just keep "
                        "simulating forward (the robot stays wherever it ends up, continuing to "
                        "act on an increasingly stale comparison to the reference). Useful for "
                        "watching contact/impact behavior over a long continuous stretch without "
                        "the view snapping back to a re-synced start pose every time it falls.")
    p.add_argument("--gui_panel", action="store_true",
                   help="show PyBullet's on-screen slider panel (reference-ghost opacity, render "
                        "delay) -- hidden by default. Doesn't affect camera: drag with the mouse "
                        "to orbit/pan/zoom regardless of this flag.")
    p.add_argument("--recovery_motion", type=str, default=None,
                   help="which --motion_file entry (exact path as listed) is the recovery motion "
                        "-- pressing R in the GUI forces the NEXT reset into a scripted fallen-on-"
                        "its-back start for it specifically (works regardless of which gait is "
                        "currently selected; switches to this motion automatically). Omit to "
                        "fall back to the old single-motion behavior below.")
    p.add_argument("--recovery_init_prob", type=float, default=0.0,
                   help="probability of starting EVERY episode from a scripted fallen-on-its-back "
                        "pose automatically, instead of only on demand via the R key -- only "
                        "meaningful when --motion_file is a single file (old behavior) or "
                        "--recovery_motion is set. 1.0 forces it every reset.")
    p.add_argument("--urdf_filename", type=str, default=None,
                   help="override od.OD's URDF (e.g. quadruped_envs/urdf/od_capsule_debug.urdf "
                        "to see the collision capsules drawn as the visual shapes too). Passed "
                        "via functools.partial, NOT by reassigning od.URDF_FILENAME -- od.OD's "
                        "own urdf_filename=URDF_FILENAME default is bound once at class-"
                        "definition time, so a post-import monkeypatch of the module attribute is "
                        "silently ignored (cost real time to learn this -- see project memory).")
    args = p.parse_args()

    import functools
    from quadruped_envs.od_imitation_env import build_imitation_env
    from quadruped_envs import od
    robot_class = od.OD if args.urdf_filename is None else functools.partial(od.OD, urdf_filename=args.urdf_filename)
    from quadruped_envs.supermimic.mdp import NUM_JOINTS, OBS_DIM, XI_DIM, State, mat_state_from, obs as mdp_obs
    from quadruped_envs.supermimic.motion import CONTROL_DT, MultiMotion
    from quadruped_envs.supermimic.networks import Policy

    motion_files = [f.strip() for f in args.motion_file.split(",")]
    cycle_len = len(motion_files)  # N/P only cycles over these; an auto-appended --recovery_motion
    # (see below) is reachable via R but deliberately excluded from this count, so spamming N/P
    # can never accidentally land on it.

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    ck_args = ck["args"]
    K, Hh = ck_args["goal_frames"], ck_args["hist"]
    print(f"checkpoint iteration={ck['iteration']}  motions={[os.path.basename(f) for f in motion_files]}")

    # Must match whatever --pi_ref_pd* the checkpoint was actually trained with (saved in
    # ck_args) -- Policy.forward()'s extra PD-bias term changes a registered buffer
    # (_ref_pd_default_q), so a mismatched reconstruction fails load_state_dict outright
    # (strict=True by default) rather than silently using the wrong behavior.
    ref_pd = None
    if ck_args.get("pi_ref_pd"):
        from quadruped_envs.imitation_wrapper_env.od_motor import TORQUE_MULTIPLIER
        ref_pd = (ck_args["pi_ref_pd_kp"], ck_args["pi_ref_pd_kd"], float(TORQUE_MULTIPLIER), ck_args["pi_ref_pd_clip"])
    pi = Policy(OBS_DIM, 39 * K, hist=Hh, ref_pd=ref_pd)
    pi.load_state_dict(ck["pi"])
    pi.eval()

    # Must match whatever --control_mode the checkpoint was actually trained with (saved in
    # ck_args) -- a position-trained policy's actions are calibrated to become joint-position
    # targets (see mdp.py's ACTION_POS_SCALE / env.py's worker), NOT raw torques; stepping them
    # into a torque-mode env would silently misinterpret them and the robot would just collapse,
    # not reflecting the checkpoint's real training quality.
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

    # recovery_motion_names flags which motion gets the scripted-fallen-start/relaxed-
    # termination treatment (see imitation_task.py) -- --recovery_motion names it explicitly for
    # the multi-gait case; falls back to the old "the single --motion_file is the recovery one"
    # behavior when there's exactly one file and --recovery_init_prob is set, unchanged. MUST run
    # (and finish any motion_files.append) BEFORE constructing MultiMotion below -- it snapshots
    # motion_files into a fixed-length internal list at construction time, so an append afterward
    # would leave MultiMotion with no entry for the new index at all (confirmed: silent
    # TypeError, every row's mask misses and frame_state's `out` never gets allocated).
    if args.recovery_motion:
        recovery_motion_names = {os.path.basename(args.recovery_motion)}
        matches = [i for i, f in enumerate(motion_files) if os.path.basename(f) == os.path.basename(args.recovery_motion)]
        if matches:
            recovery_idx = matches[0]
        else:
            # Not in --motion_file's own list -- append it rather than erroring out, so R can
            # still reach it on demand without cluttering the N/P cycling order with a motion
            # the user didn't ask to walk through normally.
            motion_files.append(args.recovery_motion)
            recovery_idx = len(motion_files) - 1
            print(f"[info] --recovery_motion {os.path.basename(args.recovery_motion)} wasn't in "
                 f"--motion_file -- appended it (reachable via R, not included in N/P cycling)")  # fmt: skip
    elif len(motion_files) == 1 and args.recovery_init_prob > 0:
        recovery_motion_names = {os.path.basename(motion_files[0])}
        recovery_idx = 0
    else:
        recovery_motion_names = frozenset()
        recovery_idx = None

    motion = MultiMotion(motion_files)
    env = build_imitation_env(motion_files=motion_files, num_parallel_envs=1,
                              enable_rendering=True, enable_randomizer=False, robot_class=robot_class,
                              enable_rendering_gui=args.gui_panel,
                              recovery_motion_names=recovery_motion_names,
                              recovery_init_prob=args.recovery_init_prob,
                              motor_control_mode=motor_control_mode)
    if args.seed is not None:
        env.seed(args.seed)
    task = env._task

    # Interactive gait control (N/P cycle, R manual reset -- see module docstring): override the
    # task's own random motion-sampling with a fixed, script-controlled index instead of touching
    # imitation_task.py itself, so every other reset mechanic (origin-offset resync, warmup,
    # recovery fallen-pose override, etc.) runs completely unchanged -- this only replaces WHICH
    # index that existing machinery picks, each time env.reset() is called.
    gait_state = {"idx": 0, "force_recovery_once": False}

    def _pick_motion_idx():
        if gait_state["force_recovery_once"] and recovery_idx is not None:
            gait_state["force_recovery_once"] = False
            return recovery_idx
        return gait_state["idx"]

    task._sample_ref_motion = _pick_motion_idx

    env.reset()
    pyb = env._pybullet_client
    sim_model = env.robot.quadruped

    gait_names = [os.path.basename(f) for f in motion_files]
    controls_text = (
        "N: next gait | P: prev gait" + (" | R: force recovery start" if recovery_idx is not None else " | R: reset")
    )
    # PyBullet's debug text has no screen-fixed/HUD mode -- it's always a 3D world object, so a
    # text anchored to a fixed WORLD point drifts across the view as soon as the robot walks away
    # from it (or the user orbits the camera -- unavoidable without redrawing every frame, which
    # is exactly the per-frame debug-draw pattern that desyncs this Mac's GUI, see module note).
    # Parenting to the robot's own base link instead keeps both labels floating just above it like
    # a nameplate, so they stay with whatever you're actually watching -- the best available
    # middle ground, though panning/orbiting the camera yourself will still move them on screen.
    TEXT_OFFSET = [0, 0, 0.55]
    pyb.addUserDebugText(controls_text, textPosition=[TEXT_OFFSET[0], TEXT_OFFSET[1], TEXT_OFFSET[2] + 0.15],
                        textColorRGB=[1, 1, 1], textSize=1.3, lifeTime=0,
                        parentObjectUniqueId=sim_model, parentLinkIndex=-1)  # fmt: skip
    status_text_id = pyb.addUserDebugText(f"gait: {gait_names[gait_state['idx']]}", textPosition=TEXT_OFFSET,
                                          textColorRGB=[1, 1, 0], textSize=1.5, lifeTime=0,
                                          parentObjectUniqueId=sim_model, parentLinkIndex=-1)  # fmt: skip
    print(f"[controls] {controls_text}")

    def _redraw_status():
        nonlocal status_text_id
        status_text_id = pyb.addUserDebugText(
            f"gait: {gait_names[gait_state['idx']]}", textPosition=TEXT_OFFSET,
            textColorRGB=[1, 1, 0], textSize=1.5, lifeTime=0, replaceItemUniqueId=status_text_id,
            parentObjectUniqueId=sim_model, parentLinkIndex=-1)

    obs_hist = torch.zeros(1, Hh, OBS_DIM)
    a_prev = torch.zeros(1, NUM_JOINTS)
    # Nominal sim-context: this script drives a single, non-randomized PyBullet instance (not
    # one of TrackingSim's workers, which are the only place env.py actually randomizes friction/
    # joint-offset -- see its module note), so there's no "real" xi to read here. Friction set to
    # the midpoint of training's randomization range (not 0, which would be a physically
    # nonsensical out-of-distribution value for the policy); joint offset 0 ("no miscalibration")
    # is already a sensible nominal value.
    xi = torch.zeros(1, XI_DIM)
    xi[:, 0] = 0.85

    step = 0
    try:
        while args.steps == 0 or step < args.steps:
            t0 = time.time()

            # Interactive controls: KEY_WAS_TRIGGERED fires once per physical key-down, not every
            # frame it's held, so this is cheap to poll every iteration.
            keys = pyb.getKeyboardEvents()
            if (ord("n") in keys and keys[ord("n")] & pyb.KEY_WAS_TRIGGERED) or \
               (ord("p") in keys and keys[ord("p")] & pyb.KEY_WAS_TRIGGERED):
                step_dir = 1 if ord("n") in keys and keys[ord("n")] & pyb.KEY_WAS_TRIGGERED else -1
                gait_state["idx"] = (gait_state["idx"] + step_dir) % cycle_len
                # Live reference swap, NOT a scene reset: _reset_ref_motion() re-anchors the new
                # gait's origin/heading/clock to the robot's CURRENT pose (read straight from the
                # simulator, see _get_sim_base_position/_get_sim_base_rotation) but deliberately
                # skips _sync_sim_model() -- the piece of ImitationTask.reset() that actually
                # teleports the robot -- so it keeps moving exactly as it was and simply starts
                # being judged against (and PD-targeted toward) the new gait from the next step,
                # picking it up on the fly instead of snapping back to a reference start pose.
                # obs_hist/a_prev are NOT cleared either, for the same reason: nothing about the
                # robot's own continuous state actually changed.
                task._reset_ref_motion()
                _redraw_status()
                print(f"[gait] -> {gait_names[gait_state['idx']]} (live switch, no reset)")
            if ord("r") in keys and keys[ord("r")] & pyb.KEY_WAS_TRIGGERED:
                if recovery_idx is not None:
                    gait_state["idx"] = recovery_idx
                    gait_state["force_recovery_once"] = True
                env.reset()  # a REAL reset -- R's whole point is teleporting to a fallen (or, with
                # no --recovery_motion, a fresh reference-synced) pose, which genuinely requires
                # moving the robot, unlike N/P above.
                obs_hist.zero_()
                a_prev.zero_()
                _redraw_status()
                print(f"[gait] -> {gait_names[gait_state['idx']]} (full reset)")

            motion_idx = torch.tensor([gait_state["idx"]], dtype=torch.long)
            t = torch.tensor([task._get_motion_time()], dtype=torch.float32)
            ref = motion.frame_state(motion_idx, t)

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
            o = mdp_obs(S_mat, a_prev, ref.R, xi)
            obs_hist = torch.cat((obs_hist[:, 1:], o.unsqueeze(1)), 1)
            g = motion.goal(motion_idx, t, K, CONTROL_DT)

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
            step += 1
            if done and not args.no_reset:
                env.reset()
                obs_hist.zero_()
                a_prev.zero_()

            elapsed = time.time() - t0
            if elapsed < CONTROL_DT:
                time.sleep(CONTROL_DT - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
