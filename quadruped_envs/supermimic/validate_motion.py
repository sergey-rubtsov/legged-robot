"""Phase-1 validation: check that ReferenceMotion.state_at(t) (motion.py) reproduces the
same root pose the real ImitationTask drives its reference playback model to, at the same
simulation time."""
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from quadruped_envs.od_imitation_env import build_imitation_env
from quadruped_envs import od
from quadruped_envs.supermimic.motion import ReferenceMotion

motion_file = os.path.join(REPO_ROOT, "quadruped_envs", "data", "motions", "od", "pace.txt")
env = build_imitation_env(motion_files=[motion_file], num_parallel_envs=1,
                           enable_rendering=False, enable_randomizer=False, robot_class=od.OD)
env.reset()

task = env._task
ref_motion = ReferenceMotion([motion_file])

rng = np.random.default_rng(1)
max_err = 0.0
for i in range(200):
    env.step(rng.uniform(env.action_space.low, env.action_space.high))
    t = task._get_motion_time()
    mine = ref_motion.state_at(t)
    real_pos, real_rot = env._pybullet_client.getBasePositionAndOrientation(task.get_ref_model())
    err = max(np.abs(mine.root_pos.numpy() - np.array(real_pos)).max(),
             np.abs(mine.root_rot.numpy() - np.array(real_rot)).max())
    max_err = max(max_err, err)
    if env.env_step_counter == 0:
        env.reset()

print("max root pos/rot error between ReferenceMotion.state_at(t) and the real ref_model:", max_err)
assert max_err < 1e-5
print("MOTION VALIDATION PASSED")
env.close()
