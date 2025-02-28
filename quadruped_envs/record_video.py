import argparse
import os

from stable_baselines3.common.vec_env import VecVideoRecorder

from quadruped_envs.utils.exp_manager import ExperimentManager
from quadruped_envs.utils.utils import ALGOS, create_test_env, get_latest_run_id, get_saved_hyperparams

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", help="environment ID", type=str, default="OpenDynamicImitationEnv-v0")
    parser.add_argument("-f", "--folder", help="Log folder", type=str, default="quadruped_envs/logs")
    parser.add_argument("-o", "--output-folder", help="Output folder", type=str, default="quadruped_envs/logs/videos/")
    parser.add_argument("--algo", help="RL Algorithm", default="ppo", type=str, required=False, choices=list(ALGOS.keys()))
    parser.add_argument("-n", "--n-timesteps", help="number of timesteps", default=1000, type=int)
    parser.add_argument("--n-envs", help="number of environments", default=1, type=int)
    parser.add_argument("--deterministic", action="store_true", default=False, help="Use deterministic actions")
    parser.add_argument("--seed", help="Random generator seed", type=int, default=0)
    parser.add_argument(
        "--no-render", action="store_true", default=False, help="Do not render the environment (useful for tests)"
    )
    parser.add_argument("--exp-id", help="Experiment ID (default: 0: latest, -1: no exp folder)", default=0, type=int)
    parser.add_argument(
        "--load-best", action="store_true", default=False, help="Load best model instead of last model if available"
    )
    parser.add_argument(
        "--load-checkpoint",
        type=int,
        help="Load checkpoint instead of last model if available, you must pass the number of timesteps corresponding to it",
    )
    args = parser.parse_args()

    env_id = args.env
    algo = args.algo
    folder = args.folder
    video_folder = args.output_folder
    seed = args.seed
    deterministic = args.deterministic
    video_length = args.n_timesteps
    n_envs = args.n_envs
    load_best = args.load_best
    load_checkpoint = args.load_checkpoint

    abs_path = os.path.abspath(os.getcwd())
    folder = os.path.join(abs_path, folder)
    if args.exp_id == 0:
        args.exp_id = get_latest_run_id(os.path.join(folder, algo), env_id)
        print(f"Loading latest experiment, id={args.exp_id}")
    # Sanity checks
    if args.exp_id > 0:
        log_path = os.path.join(folder, algo, f"{env_id}_{args.exp_id}")
    else:
        log_path = os.path.join(folder, algo)

    assert os.path.isdir(log_path), f"The {log_path} folder was not found"

    if load_best:
        model_path = os.path.join(log_path, "best_model.zip")
    elif load_checkpoint is None:
        # Default: load latest model
        model_path = os.path.join(log_path, f"{env_id}.zip")
    else:
        model_path = os.path.join(log_path, f"rl_model_{args.load_checkpoint}_steps.zip")

    found = os.path.isfile(model_path)
    if not found:
        raise ValueError(f"No model found for {algo} on {env_id}, path: {model_path}")

    stats_path = os.path.join(log_path, env_id)
    hyperparams, stats_path = get_saved_hyperparams(stats_path)

    is_atari = ExperimentManager.is_atari(env_id)

    env = create_test_env(
        env_id,
        n_envs=n_envs,
        stats_path=stats_path,
        seed=seed,
        log_dir=None,
        should_render=not args.no_render,
        hyperparams=hyperparams,
    )

    model = ALGOS[algo].load(model_path)

    obs = env.reset()

    # Note: apparently it renders by default
    env = VecVideoRecorder(
        env,
        video_folder,
        record_video_trigger=lambda x: x == 0,
        video_length=video_length,
        name_prefix=f"{algo}-{env_id}",
    )

    env.reset()
    try:
        for _ in range(video_length + 1):
            action, _ = model.predict(obs, deterministic=deterministic)
            obs, _, _, _ = env.step(action)
            if not args.no_render:
                env.render()
    except KeyboardInterrupt:
        pass

    env.close()
