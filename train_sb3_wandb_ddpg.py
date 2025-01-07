import argparse
from omni.isaac.lab.app import AppLauncher
import gymnasium as gym
import numpy as np
import torch.nn as nn  # Import nn to access activation functions
from stable_baselines3 import DDPG, HerReplayBuffer
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.noise import NormalActionNoise

# argparse for non-agent parameters
parser = argparse.ArgumentParser(description="Train an RL agent with Stable-Baselines3.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to the model checkpoint.")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
args_cli.headless = True  # Set this based on your requirement

# launch the omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gym_env.env  # Ensure custom environment is recognized
from omni.isaac.lab_tasks.utils.parse_cfg import parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.sb3 import Sb3VecEnvWrapper
import wandb
import yaml
from wandb.integration.sb3 import WandbCallback


def main():
    """Train with stable-baselines agent."""
    # WandB initialization (config.yaml values come from WandB during sweep)
    with open("./config_sb3_ddpg.yaml") as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    run = wandb.init(
        project="rel_ik_sb3_ddpg_ur5e_lift_cube_0_05",
        config=config,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=False,  # auto-upload the videos of agents playing the game
        save_code=False,  # Save code for reproducibility
    )

    # Load env cfg
    task = "UR5e-Lift-Cube-IK"
    num_envs = 4096
    device = "cuda"
    env_cfg = parse_env_cfg(task, device=device, num_envs=num_envs)
    env_cfg.seed = wandb.config["seed"]

    # Create Isaac environment
    env = gym.make(task, cfg=env_cfg, render_mode=None)

    # Wrap around environment for stable baselines
    env = Sb3VecEnvWrapper(env)

    # Normalization wrappers based on agent configuration
    if wandb.config["normalize_input"]:
        env = VecNormalize(
            env,
            training=True,
            norm_obs=wandb.config["normalize_input"],
            norm_reward=wandb.config["normalize_value"],
            clip_obs=wandb.config["clip_obs"],
            gamma=wandb.config["gamma"],
            clip_reward=np.inf,
        )

    # Prepare policy kwargs
    policy_kwargs = wandb.config.policy_kwargs

    # Map string activation function to actual callable
    if 'activation_fn' in policy_kwargs:
        activation_fn_name = policy_kwargs['activation_fn']
        
        # Map string name to the actual function
        if activation_fn_name == 'nn.ELU':
            policy_kwargs['activation_fn'] = nn.ELU
        elif activation_fn_name == 'nn.ReLU':
            policy_kwargs['activation_fn'] = nn.ReLU
        elif activation_fn_name == 'nn.Tanh':
            policy_kwargs['activation_fn'] = nn.Tanh
        else:
            raise ValueError(f"Unknown activation function: {activation_fn_name}")
        
    if "action_noise" in wandb.config and wandb.config["action_noise"] == "NormalActionNoise":
        action_dim = env.action_space.shape[0]
        action_noise = NormalActionNoise(mean=np.zeros(action_dim), sigma=0.1 * np.ones(action_dim))
    else:
        action_noise = None


    # HER Replay Buffer (if enabled)
    # if "replay_buffer_class" in wandb.config and wandb.config["replay_buffer_class"] == "HerReplayBuffer":
    #     replay_buffer_kwargs = eval(wandb.config["replay_buffer_kwargs"])
    #     replay_buffer_class = HerReplayBuffer
    # else:
    #     replay_buffer_class = None
    #     replay_buffer_kwargs = None


    # Create a new agent from stable baselines
    agent = DDPG(
        wandb.config["policy"], 
        env, 
        verbose=1, 
        tensorboard_log=f"runs/{run.id}", 
        batch_size=wandb.config.batch_size,
        gamma=wandb.config.gamma,
        learning_rate=wandb.config.learning_rate,
        train_freq=wandb.config.train_freq,
        gradient_steps=wandb.config.gradient_steps,
        buffer_size=wandb.config.buffer_size,
        learning_starts=wandb.config.learning_starts,
        tau=wandb.config.tau,
        policy_kwargs=policy_kwargs,
        action_noise=action_noise,
        # replay_buffer_class=replay_buffer_class,
        # replay_buffer_kwargs=replay_buffer_kwargs,
    )

    # Train the agent
    agent.learn(
        total_timesteps=wandb.config["n_timesteps"],
        callback=WandbCallback(
            gradient_save_freq=10000,
            model_save_path=f"models/{run.id}",
            verbose=2,
        ),
    )


    # Close the environment
    env.close()
    run.finish()


if __name__ == "__main__":
    main()
    simulation_app.close()
