"""Script a smooth, controlled 'lie down' motion (idle standing pose -> a tucked, roughly
sternal-recumbent pose) by directly commanding joint angles via POSITION control and letting
real physics (gravity, contact) settle the robot -- NOT trained with RL, since this is meant to
be a clean, deliberately slow and symmetric reference, not something an RL reward might find a
chaotic/asymmetric way to achieve. Records the real resulting trajectory (not just the commanded
one) and saves it in the same DeepMimic-JSON schema as quadruped_envs/data/motions/od/*.txt, so
it can be used as an ordinary --motion_file entry.

This is step 1 of the planned "get up" pipeline (see project memory): record this, then a
separate script time-reverses it into a stand-up reference, on the theory that a trajectory the
robot can actually traverse forward (under active control) is also a kinematically valid
(if not necessarily dynamically ideal -- see the velocity-profile caveat discussed with the
user) sequence of poses to imitate in reverse.

Usage: python -m quadruped_envs.supermimic.make_lie_down_motion --out quadruped_envs/data/motions/od/lie_down.txt
Add --render to watch it live while generating.
"""
import argparse
import json
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

NUM_JOINTS = 12
# Per-leg [hip, upper, knee] target for the tucked/lying pose, same for all 4 legs (legs a-d
# use mirrored sign conventions for hip only -- see od_robot.py's docstring/JOINT_DIRECTIONS in
# od.py -- upper/knee share the same convention across legs). Hip stays near its standing value
# (don't splay sideways -- keeps the descent symmetric and easy to reverse cleanly). Upper leg
# and knee fold deeply to lower the body -- signs/magnitudes picked empirically, see the
# --render check this script's docstring recommends before trusting the result.
LIE_HIP = 0.0
LIE_UPPER = 1.3
LIE_KNEE = -2.4


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--start_motion", type=str,
                   default=os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "idle.txt"))
    p.add_argument("--out", type=str,
                   default=os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "lie_down.txt"))
    p.add_argument("--descend_time", type=float, default=3.0, help="seconds to interpolate from standing to the lying target")
    p.add_argument("--settle_time", type=float, default=2.0, help="extra seconds to hold/settle at the end")
    p.add_argument("--render", action="store_true")
    p.add_argument("--urdf_filename", type=str, default=None,
                   help="override od.OD's default URDF (od.py's own URDF_FILENAME constant) -- "
                        "note this must be passed as an explicit constructor argument, not set by "
                        "monkeypatching od.URDF_FILENAME after import: OD.__init__'s own "
                        "urdf_filename=URDF_FILENAME default is bound once at class-definition "
                        "time, so reassigning the module attribute afterward has no effect on it.")
    args = p.parse_args()

    import pybullet
    import pybullet_data
    from motion_imitation.robots import robot_config
    from quadruped_envs import od

    frame_duration = 1.0 / 60.0  # matches the existing motion files' 60fps convention
    sim_time_step = 0.005
    substeps_per_frame = max(1, round(frame_duration / sim_time_step))

    client = pybullet.connect(pybullet.GUI if args.render else pybullet.DIRECT)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -10)
    pybullet.setTimeStep(sim_time_step)
    pybullet.setPhysicsEngineParameter(numSolverIterations=50, useSplitImpulse=1,
                                       splitImpulsePenetrationThreshold=-0.01, contactERP=0.8)
    pybullet.loadURDF("plane_implicit.urdf")

    od_kwargs = dict(pybullet_client=pybullet, time_step=sim_time_step, action_repeat=1,
                     motor_control_mode=robot_config.MotorControlMode.POSITION)
    if args.urdf_filename is not None:
        od_kwargs["urdf_filename"] = args.urdf_filename
    robot = od.OD(**od_kwargs)

    start = json.load(open(args.start_motion))
    start_frame = np.array(start["Frames"][0], dtype=np.float64)
    start_root_pos, start_root_quat, start_q = start_frame[:3], start_frame[3:7], start_frame[7:19]

    pybullet.resetBasePositionAndOrientation(robot.quadruped, start_root_pos.tolist(), start_root_quat.tolist())
    for j in range(NUM_JOINTS):
        # Joint index j IS the pybullet joint index directly for this URDF -- confirmed by every
        # other script in this package (eval_policy_rollout.py, diagnose_rollout.py, etc.) using
        # getJointStateMultiDof(sim_model, j) the same way, no separate id-lookup table.
        pybullet.resetJointState(robot.quadruped, j, start_q[j], 0.0)

    target_q = np.array([LIE_HIP, LIE_UPPER, LIE_KNEE] * 4, dtype=np.float64)
    n_descend = max(1, round(args.descend_time / frame_duration))
    n_settle = max(1, round(args.settle_time / frame_duration))
    n_total = n_descend + n_settle

    recorded = []

    def record_frame():
        pos, quat = pybullet.getBasePositionAndOrientation(robot.quadruped)
        q = [pybullet.getJointStateMultiDof(robot.quadruped, j)[0][0] for j in range(NUM_JOINTS)]
        recorded.append(list(pos) + list(quat) + q)

    record_frame()  # frame 0: the exact starting pose
    for i in range(1, n_total + 1):
        alpha = min(1.0, i / n_descend)
        smooth_alpha = alpha * alpha * (3 - 2 * alpha)  # smoothstep: zero velocity at both ends
        desired_q = (1 - smooth_alpha) * start_q + smooth_alpha * target_q
        for _ in range(substeps_per_frame):
            robot.Step(desired_q, control_mode=robot_config.MotorControlMode.POSITION)
        record_frame()

    out = {
        "LoopMode": "Clamp",
        "FrameDuration": frame_duration,
        "EnableCycleOffsetPosition": False,
        "EnableCycleOffsetRotation": False,
        "Frames": [[round(float(x), 5) for x in frame] for frame in recorded],
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    json.dump(out, open(args.out, "w"), indent=2)
    print(f"saved {len(recorded)} frames ({len(recorded) * frame_duration:.2f}s) -> {args.out}")
    final_pos, _ = pybullet.getBasePositionAndOrientation(robot.quadruped)
    print(f"final root height: {final_pos[2]:.4f} m (standing was {start_root_pos[2]:.4f} m)")
    pybullet.disconnect()


if __name__ == "__main__":
    main()
