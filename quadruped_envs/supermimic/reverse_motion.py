"""Time-reverse a DeepMimic-JSON motion clip (same schema as quadruped_envs/data/motions/od/*.txt)
to produce a new reference clip. Used to turn the scripted 'lie down' motion
(make_lie_down_motion.py) into a 'get up' reference: a trajectory the robot can actually
traverse forward under active control is, played backwards, a kinematically valid sequence of
poses to imitate in the other direction -- see the project memory / conversation for the caveats
(the resulting velocity profile is the time-mirror of the forward one, which may not be the most
dynamically favorable timing for standing up, and the pose sequence's own contact order is also
mirrored -- both worth checking empirically against the trained result, not assumed correct).

Reversing is just reversing the Frames list: velocities aren't stored in the file at all (every
loader in this project, motion_imitation's MotionData included, computes them from consecutive
frames' positions at load time), so a reversed frame order alone gives correct reversed
velocities for free -- no separate sign-flip or recomputation needed.

Usage: python -m quadruped_envs.supermimic.reverse_motion \
         --in quadruped_envs/data/motions/od/lie_down.txt \
         --out quadruped_envs/data/motions/od/get_up.txt
"""
import argparse
import json


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="in_path", type=str, required=True)
    p.add_argument("--out", type=str, required=True)
    args = p.parse_args()

    d = json.load(open(args.in_path))
    d["Frames"] = list(reversed(d["Frames"]))
    json.dump(d, open(args.out, "w"), indent=2)
    print(f"reversed {len(d['Frames'])} frames: {args.in_path} -> {args.out}")


if __name__ == "__main__":
    main()
