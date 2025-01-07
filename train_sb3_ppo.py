import argparse
import sys

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with Stable-Baselines3.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to the model checkpoint.")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
parser.add_argument("--no_logging", action="store_true", default=False, help="Disable logging for the training process.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import numpy as np
import os
from datetime import datetime

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import VecNormalize

from omni.isaac.lab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from omni.isaac.lab.utils.dict import print_dict
from omni.isaac.lab.utils.io import dump_pickle, dump_yaml

import gym_env.env # This import is strictly necessary otherwise it would not recognize the registered custom gym environment
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.sb3 import Sb3VecEnvWrapper, process_sb3_cfg




@hydra_task_config(args_cli.task, "sb3_ppo_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: dict):
    """Train with stable-baselines agent."""
    # override configurations with non-hydra CLI arguments
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg["seed"] = args_cli.seed if args_cli.seed is not None else agent_cfg["seed"]

    # max iterations for training
    if args_cli.max_iterations is not None:
        agent_cfg["n_timesteps"] = args_cli.max_iterations * agent_cfg["n_steps"] * env_cfg.scene.num_envs

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg["seed"]
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # Directory for logging into
    if not args_cli.no_logging:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir, "logs", "sb3", "ppo", args_cli.task, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        # dump the configuration into log-directory
        dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
        dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
        dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
        dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)
    else:
        log_dir = None  # No logging
    

    # post-process agent configuration
    agent_cfg = process_sb3_cfg(agent_cfg)
    # read configurations about the agent-training
    policy_arch = agent_cfg.pop("policy")
    n_timesteps = agent_cfg.pop("n_timesteps")

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    
    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "train"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)  

    # wrap around environment for stable baselines
    env = Sb3VecEnvWrapper(env)

    if "normalize_input" in agent_cfg:
        env = VecNormalize(
            env,
            training=True,
            norm_obs="normalize_input" in agent_cfg and agent_cfg.pop("normalize_input"),
            norm_reward="normalize_value" in agent_cfg and agent_cfg.pop("normalize_value"),
            clip_obs="clip_obs" in agent_cfg and agent_cfg.pop("clip_obs"),
            gamma=agent_cfg["gamma"],
            clip_reward=np.inf,
        )

    # Load or create a new agent from stable baselines
    if args_cli.checkpoint is not None:
        print(f"Loading checkpoint from: {args_cli.checkpoint}")
        agent = PPO.load(args_cli.checkpoint, env)  # Load the model from the checkpoint
    else:
        agent = PPO(policy_arch, env, verbose=1, **agent_cfg)  # Create a new model


    # Configure the logger only if logging is enabled
    if not args_cli.no_logging:
        new_logger = configure(log_dir, ["stdout", "csv", "tensorboard"])
        agent.set_logger(new_logger)

        checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=log_dir, name_prefix="model", verbose=2)
    else:
        checkpoint_callback = None
    
    # train the agent
    agent.learn(total_timesteps=n_timesteps, callback=checkpoint_callback)





    # # Initialize the environment and variables
    # obs = env.reset()
    # total_timesteps = n_timesteps
    # timestep = 0
    # rollout_buffer = []  # Buffer to collect rollouts for training

    # while timestep < total_timesteps:
    #     # Predict the action using the policy
    #     action, _states = agent.predict(obs, deterministic=False)

    #     print(f"Timestep: {timestep}, Action: {action}")

    #     # Take the action in the environment
    #     new_obs, reward, done, info = env.step(action)

    #     # Print the reward for this timestep
    #     # print(f"Timestep: {timestep}, Reward: {reward}")
    #     print(f"Timestep: {timestep}, Obs: {obs}")


    #     # Collect data for learning
    #     rollout_buffer.append((obs, action, reward, new_obs, done))

    #     # Update observation
    #     obs = new_obs

    #     # If any of the environments are done, reset them
    #     if np.any(done):  # Use np.any to handle the array
    #         obs = env.reset()

    #     # Increment the timestep
    #     timestep += 1

    #     # Periodically train the agent using the collected rollouts
    #     if len(rollout_buffer) >= agent.n_steps:  # `agent.n_steps` is the number of steps per update
    #         for experience in rollout_buffer:
    #             obs, action, reward, new_obs, done = experience
    #             agent.policy.optimizer.zero_grad()
    #             agent.policy.optimizer.step()  # Perform the training step
    #         rollout_buffer.clear()  # Clear buffer after training








    # Save the final model
    if not args_cli.no_logging:
        agent.save(os.path.join(log_dir, "model"))

    # Close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()