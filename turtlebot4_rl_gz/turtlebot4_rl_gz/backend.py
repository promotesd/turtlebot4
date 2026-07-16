"""Gazebo Harmonic implementation of the world-control port."""

from __future__ import annotations

import math
import time
from typing import Any

from geometry_msgs.msg import Pose
import numpy as np
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity, SetEntityPose, SpawnEntity

from turtlebot4_rl_core import Goal2D
from turtlebot4_rl_gz.config import GazeboWorldConfig
from turtlebot4_rl_gz.model import goal_marker_sdf


class GazeboWorldBackend:
    """Reset robot/goal entities without leaking Gazebo into algorithms."""

    def __init__(self, config: GazeboWorldConfig | None = None, node: Node | None = None) -> None:
        self.config = config or GazeboWorldConfig()
        if node is None and not rclpy.ok():  # type: ignore[attr-defined]
            rclpy.init()
        self._owns_node = node is None
        self.node = node or Node('turtlebot4_rl_gazebo_backend')
        prefix = f'/world/{self.config.world_name}'
        self._control = self.node.create_client(ControlWorld, prefix + '/control')
        self._set_pose = self.node.create_client(SetEntityPose, prefix + '/set_pose')
        self._spawn = self.node.create_client(SpawnEntity, prefix + '/create')
        self._delete = self.node.create_client(DeleteEntity, prefix + '/remove')
        self._goal = Goal2D(self.config.goal_min_x, 0.0)
        self._closed = False

    @property
    def goal(self) -> Goal2D:
        return self._goal

    def _call(self, client: Any, request: Any, label: str, *, required: bool = True) -> Any:
        timeout = self.config.service_timeout_seconds
        if not client.wait_for_service(timeout_sec=timeout):
            if required:
                raise TimeoutError(f'Gazebo service unavailable: {label}')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            if required:
                raise TimeoutError(f'Gazebo service timed out: {label}')
            return None
        return future.result()

    def _pause(self, paused: bool) -> None:
        request = ControlWorld.Request()
        request.world_control.pause = paused
        result = self._call(self._control, request, 'control')
        if not result.success:
            raise RuntimeError('Gazebo rejected world pause/resume')

    @staticmethod
    def _pose(x: float, y: float, yaw: float = 0.0) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _set_robot_pose(self, x: float, y: float, yaw: float) -> None:
        request = SetEntityPose.Request()
        request.entity.name = self.config.robot_entity_name
        request.entity.type = Entity.MODEL
        request.pose = self._pose(x, y, yaw)
        result = self._call(self._set_pose, request, 'set_pose')
        if not result.success:
            raise RuntimeError('Gazebo rejected robot pose reset')

    def _replace_goal_marker(self) -> None:
        delete = DeleteEntity.Request()
        delete.entity.name = self.config.goal_entity_name
        delete.entity.type = Entity.MODEL
        self._call(self._delete, delete, 'remove', required=False)
        spawn = SpawnEntity.Request()
        spawn.entity_factory.name = self.config.goal_entity_name
        spawn.entity_factory.allow_renaming = False
        spawn.entity_factory.pose = self._pose(self._goal.x, self._goal.y)
        spawn.entity_factory.sdf = goal_marker_sdf()
        result = self._call(self._spawn, spawn, 'create')
        if not result.success:
            raise RuntimeError('Gazebo rejected goal marker spawn')

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, object] | None = None,
    ) -> dict[str, object]:
        if self._closed:
            raise RuntimeError('Gazebo world backend is closed')
        options = options or {}
        rng = np.random.default_rng(seed)
        if 'goal' in options:
            goal = options['goal']
            if not isinstance(goal, (tuple, list)) or len(goal) != 2:
                raise ValueError('goal must contain x and y')
            self._goal = Goal2D(float(goal[0]), float(goal[1]))
        else:
            self._goal = Goal2D(
                float(rng.uniform(self.config.goal_min_x, self.config.goal_max_x)),
                float(rng.uniform(self.config.goal_min_y, self.config.goal_max_y)),
            )
        self._pause(True)
        try:
            self._set_robot_pose(
                self.config.robot_start_x,
                self.config.robot_start_y,
                self.config.robot_start_yaw,
            )
            self._replace_goal_marker()
        finally:
            self._pause(False)
        if self.config.settle_seconds:
            time.sleep(self.config.settle_seconds)
        return {'seed': seed, 'backend': 'gazebo_harmonic'}

    def set_goal(self, x: float, y: float) -> None:
        self._goal = Goal2D(float(x), float(y))
        self._replace_goal_marker()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._owns_node:
            self.node.destroy_node()  # type: ignore[no-untyped-call]
