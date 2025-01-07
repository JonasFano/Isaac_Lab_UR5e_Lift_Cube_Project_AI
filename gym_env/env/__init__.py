import gymnasium as gym
from . import agents, ik_rel_env_cfg, ik_rel_env_cfg_franka, joint_pos_env_cfg, joint_pos_env_cfg_franka

# Register Gym environments.


# Relative Differential Inverse Kinematics Action Space

gym.register(
    id="UR5e-Lift-Cube-IK",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg.RelIK_UR5e_LiftCubeEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
    },
)


# Joint Position Action Space

gym.register(
    id="UR5e-Lift-Cube",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.JointPos_UR5e_LiftCubeEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
    },
)


gym.register(
    id="Franka-Lift-Cube",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg_franka.JointPos_Franka_LiftCubeEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
    },
)


gym.register(
    id="Franka-Lift-Cube-IK",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg_franka.RelIK_Franka_LiftCubeEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
    },
)