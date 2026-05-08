# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
"""Base module for parsing kinematics from different representations.

cuRobo provides kinematics parsing from an URDF and a partial implementation for parsing from
a USD. To parse from other representations, an user can extend the :class:`~RobotParser`
class and implement only the abstract methods. Optionally, user can also provide functions for
reading meshes, useful for debugging and visualization.

"""

import abc

from curobo._src.geom.types import Mesh, Obstacle
from curobo._src.robot.types.link_params import LinkParams


class RobotParser(abc.ABC):
    def __init__(self, extra_links: dict[str, LinkParams] | None = None) -> None:
        self.extra_links = extra_links
        self.build()

    @abc.abstractmethod
    def get_link_names(self) -> list[str]: ...

    @abc.abstractmethod
    def get_joint_names(self) -> list[str]: ...

    @abc.abstractmethod
    def build(self): ...

    @abc.abstractmethod
    def get_link_parameters(self, link_name: str, base: bool = False) -> LinkParams: ...

    @abc.abstractmethod
    def get_link_mesh(self, link_name: str, use_collision_mesh: bool = False) -> Mesh | None: ...

    @abc.abstractmethod
    def get_link_geometry(
        self, link_name: str, use_collision_mesh: bool = False
    ) -> list[Obstacle]: ...

    @abc.abstractmethod
    def get_chain(self, base_link: str, ee_link: str) -> list[str]:
        """Get list of links attaching ee_link to base_link"""

    @abc.abstractmethod
    def get_controlled_joint_names(self) -> list[str]:
        """Get names of all controlled joints in the robot"""
