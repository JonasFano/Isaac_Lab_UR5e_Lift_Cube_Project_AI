# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import RigidObject
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils.math import subtract_frame_transforms, quat_apply, axis_angle_from_quat

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def quat_rotate_vector(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """
    Rotate a vector by a quaternion using provided utility functions.
    
    Args:
        quat: [..., 4] tensor representing quaternions (w, x, y, z).
        vec: [..., 3] tensor representing the vectors to rotate.
    
    Returns:
        Rotated vector of shape [..., 3].
    """
    # Ensure the quaternion is normalized to avoid unintended scaling effects
    quat = torch.nn.functional.normalize(quat, p=2, dim=-1)
    
    # Rotate the input vector using the quaternion
    rotated_vec = quat_apply(quat, vec)
    
    return rotated_vec


def get_current_tcp_pose(env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Compute the current TCP pose in both the base frame and world frame.
    
    Args:
        env: ManagerBasedRLEnv object containing the virtual environment.
        robot_cfg: Configuration for the robot entity, defaults to "robot".
    
    Returns:
        tcp_pose_b: TCP pose in the robot's base frame (position + quaternion).
    """
    # Access the robot object from the scene using the provided configuration
    robot: RigidObject = env.scene[robot_cfg.name]

    # Clone the body states in the world frame to avoid modifying the original tensor
    body_state_w_list = robot.data.body_state_w.clone()

    # Extract the pose of the end-effector (position + orientation) in the world frame
    ee_pose_w = body_state_w_list[:, robot_cfg.body_ids[0], :7]

    # Define the offset from the end-effector frame to the TCP in the end-effector frame
    offset_ee = torch.tensor([0.0, 0.0, 0.135], device="cuda").unsqueeze(0).repeat(env.scene.num_envs, 1)

    # Rotate the offset from the end-effector frame to the world frame
    offset_w = quat_rotate_vector(ee_pose_w[:, 3:7], offset_ee)

    # Compute the TCP pose in the world frame by adding the offset to the end-effector's position
    tcp_pose_w = torch.cat((ee_pose_w[:, :3] + offset_w, ee_pose_w[:, 3:7]), dim=-1)

    # Transform the TCP pose from the world frame to the robot's base frame
    tcp_pos_b, tcp_quat_b = subtract_frame_transforms(
        robot.data.root_state_w[:, :3],  # Robot base position in world frame
        robot.data.root_state_w[:, 3:7],  # Robot base orientation in world frame
        tcp_pose_w[:, :3],  # TCP position in world frame
        tcp_pose_w[:, 3:7]  # TCP orientation in world frame
    )

    # # Convert orientation from quat to axis-angle
    # tcp_axis_angle_b = axis_angle_from_quat(tcp_quat_b)
    # # Concatenate the position and axis-angle into a single tensor
    # tcp_pose_b = torch.cat((tcp_pos_b, tcp_axis_angle_b), dim=-1)

    tcp_pose_b = torch.cat((tcp_pos_b, tcp_quat_b), dim=-1)
    return tcp_pose_b


def object_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """The position of the object in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    object_pos_w = object.data.root_pos_w[:, :3]
    object_pos_b, object_quat_b = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], object_pos_w
    )
    object_pose_b = torch.cat((object_pos_b, object_quat_b), dim=-1)
    return object_pose_b