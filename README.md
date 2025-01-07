# Short project description
This project was done in the Project in Artificial Intelligence course of the Master's programme "Robot Systems - Advanced Robotics Technology" at the University of Southern Denmark (SDU). The task was to do a methodology comparison of at least two reinforcement learning algorithms.

This Repository includes the implementation to train PPO, DDPG or TD3 agents (from Stable-Baselines3) in Isaac Lab. The considered task includes a UR5e or Franka robot and requires it to lift a cube to a desired pose with differential inverse kinematics (IK) control or joint position control.
The focus of the study was on the UR5e robot controlled with a differential IK controller working with position and orientation displacements relative to the current TCP pose (= relative differential IK control). 
The Weights&Biases tool was utilized to automate the hyperparameter search since it allows to extensively visualize the episode reward mean of a range of training runs each with different hyperparameter configurations. 


# Steps to execute the code
Follow these steps to create a virtual python environment and to install Isaac Sim and Isaac Lab:

    https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html

Install requirements:
    
    pip install -r /path/to/requirements.txt 


## Use Weights&Biases for hyperparameter search
### PPO
    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube config_sb3_ppo.yaml


### DDPG
    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    wandb sweep --project rel_ik_sb3_ddpg_ur5e_lift_cube config_sb3_ddpg.yaml



### TD3
    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube config_sb3_td3.yaml



## Train PPO agent without Weights&Biases
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    python3 train_sb3_ppo.py --num_envs 4096 --task UR5e-Lift-Cube-IK --headless

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/train_sb3_ppo.py --num_envs 4096 --task UR5e-Lift-Cube-IK --headless

Tensorboard can be used to visualize training results

    tensorboard --logdir='directory'

Note: For this option, the hyperparameters are defined in /gym_env/env/agents/


## Play trained agent

### PPO
    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    python3 play_sb3_ppo.py --num_envs 4 --task UR5e-Lift-Cube-IK --checkpoint /path/to/trained/model


### DDPG
    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    python3 play_sb3_ddpg.py --num_envs 4 --task UR5e-Lift-Cube-IK --checkpoint /path/to/trained/model

### TD3
    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository
    python3 play_sb3_td3.py --num_envs 4 --task UR5e-Lift-Cube-IK --checkpoint /path/to/trained/model



## Task options (defined in /gym_env/env/__init__.py)
UR5e and Relative Differential Inverse Kinematics Action Space

    --task UR5e-Lift-Cube-IK

UR5e and Joint Position Action Space

    --task UR5e-Lift-Cube

Franka and Relative Differential Inverse Kinematics Action Space

    --task Franka-Lift-Cube-IK

Franka and Joint Position Action Space

    --task Franka-Lift-Cube

