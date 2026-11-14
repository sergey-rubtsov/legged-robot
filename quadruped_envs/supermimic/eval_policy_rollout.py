"""Deploy a training checkpoint's policy in the real (not imagined) simulator and capture a
GIF -- a qualitative look at what the policy has actually learned, as opposed to the
training-loss curves. Deterministic (no exploration noise), unlike collect() during training.

Usage: python -m quadruped_envs.supermimic.eval_policy_rollout --ckpt path/to/last.pt

--fixed_camera keeps the camera still in world space so net travel is visible; the default
chase cam keeps the robot large and the checkered floor shows motion. --cam_* apply to both.
--untrained ignores the checkpoint's policy weights (fresh random init) for a before/after
comparison, but still reads the checkpoint's args so the control mode matches.
"""
import argparse
import os
import sys

import numpy as np
import torch
from PIL import Image

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", type=str, required=True, help="path to a train.py checkpoint (last.pt or ckpt_NNNNNN.pt)")
    p.add_argument("--motion_file", type=str,
                   default=os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt"))
    p.add_argument("--steps", type=int, default=600)
    p.add_argument("--frame_stride", type=int, default=4, help="keep every Nth sim step as a GIF frame (4 -> real-time at 50 fps)")
    p.add_argument("--out", type=str, default="policy_rollout.gif")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--no_reset", action="store_true", help="don't reset on fall, keep recording")
    p.add_argument("--untrained", action="store_true", help="random-init policy instead of the checkpoint's weights")
    p.add_argument("--fixed_camera", action="store_true")
    p.add_argument("--cam_dist", type=float, default=1.3)
    p.add_argument("--cam_yaw", type=float, default=0.0)
    p.add_argument("--cam_pitch", type=float, default=-15.0)
    p.add_argument("--cam_ahead", type=float, default=1.5, help="fixed camera target offset along +x from the start")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    args = p.parse_args()

    from quadruped_envs.od_imitation_env import build_imitation_env
    from quadruped_envs import od
    from quadruped_envs.supermimic.mdp import NUM_JOINTS, OBS_DIM, XI_DIM, State, mat_state_from, obs as mdp_obs
    from quadruped_envs.supermimic.motion import CONTROL_DT, BakedMotion
    from quadruped_envs.supermimic.networks import Policy

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    ck_args = ck["args"]
    K, Hh = ck_args["goal_frames"], ck_args["hist"]
    print(f"checkpoint iteration={ck['iteration']}  K={K} hist={Hh}  untrained={args.untrained}")

    # Must match whatever --pi_ref_pd* / --control_mode the checkpoint was trained with -- see
    # demo_live.py's identical notes.
    ref_pd = None
    if ck_args.get("pi_ref_pd"):
        from quadruped_envs.imitation_wrapper_env.od_motor import TORQUE_MULTIPLIER
        ref_pd = (ck_args["pi_ref_pd_kp"], ck_args["pi_ref_pd_kd"], float(TORQUE_MULTIPLIER), ck_args["pi_ref_pd_clip"])
    if args.untrained:
        torch.manual_seed(args.seed if args.seed is not None else 0)
    pi = Policy(OBS_DIM, 39 * K, hist=Hh, ref_pd=ref_pd)
    if not args.untrained:
        pi.load_state_dict(ck["pi"])
    pi.eval()

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
    env = build_imitation_env(motion_files=[args.motion_file], num_parallel_envs=1,
                              enable_rendering=False, enable_randomizer=False, robot_class=od.OD,
                              motor_control_mode=motor_control_mode)
    if args.seed is not None:
        env.seed(args.seed)
    env.reset()
    task = env._task
    pyb = env._pybullet_client
    sim_model = env.robot.quadruped

    start_pos = np.array(pyb.getBasePositionAndOrientation(sim_model)[0])
    if args.fixed_camera:
        view_matrix = pyb.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[start_pos[0] + args.cam_ahead, start_pos[1], 0.25],
            distance=args.cam_dist, yaw=args.cam_yaw, pitch=args.cam_pitch, roll=0, upAxisIndex=2)
        proj_matrix = pyb.computeProjectionMatrixFOV(fov=60, aspect=args.width / args.height, nearVal=0.1, farVal=100.0)

        def render():
            _, _, px, _, _ = pyb.getCameraImage(width=args.width, height=args.height,
                                                viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                                                renderer=pyb.ER_TINY_RENDERER)
            return np.array(px, dtype=np.uint8).reshape(args.height, args.width, -1)[:, :, :3]
    else:
        env._camera_dist, env._camera_yaw, env._camera_pitch = args.cam_dist, args.cam_yaw, args.cam_pitch
        env._render_width, env._render_height = args.width, args.height

        def render():
            return env.render(mode="rgb_array").astype(np.uint8)

    obs_hist = torch.zeros(1, Hh, OBS_DIM)
    a_prev = torch.zeros(1, NUM_JOINTS)
    # Nominal sim-context -- see demo_live.py's identical note.
    xi = torch.zeros(1, XI_DIM)
    xi[:, 0] = 0.85
    frames = []
    falls = 0

    for step in range(args.steps):
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
        if step % args.frame_stride == 0:
            frames.append(Image.fromarray(render()))
        if done:
            falls += 1
            if not args.no_reset:
                env.reset()
                obs_hist.zero_()
                a_prev.zero_()

    end_pos = np.array(pyb.getBasePositionAndOrientation(sim_model)[0])
    print(f"falls/resets={falls}  net displacement x={end_pos[0] - start_pos[0]:+.2f} m  y={end_pos[1] - start_pos[1]:+.2f} m")
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=int(CONTROL_DT * args.frame_stride * 1000), loop=0)
    print(f"saved {args.out} ({len(frames)} frames)")
    env.close()


if __name__ == "__main__":
    main()
