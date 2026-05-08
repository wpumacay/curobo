from dataclasses import dataclass
from pathlib import Path

import mujoco as mj

from curobo._src.robot.parser.parser_base import RobotParser
from curobo._src.geom.types import Capsule, Cuboid, Cylinder, Mesh, Obstacle, Sphere
from curobo._src.robot.types.joint_types import JointType
from curobo._src.robot.types.link_params import LinkParams
from curobo._src.types.pose import Pose


class MjcfRobotParser(RobotParser):
    def __init__(self, mjcf_path: str) -> None:

        self._spec = mj.MjSpec.from_file(mjcf_path)
        self._model_dir = Path(self._spec.modelfiledir)

        self._links_params_cache: dict[str, LinkParams] = {}

        self._mesh_cache: dict[str, Mesh] = {}

        super().__init__(None)

    def get_link_names(self) -> list[str]:
        return [body.name for body in self._spec.bodies if body.name != "world"]

    def get_joint_names(self) -> list[str]:
        return [joint.name for joint in self._spec.joints if joint.type != mj.mjtJoint.mjJNT_FREE]

    def build(self) -> None:
        for mesh_spec in self._spec.meshes:
            assert isinstance(mesh_spec, mj.MjsMesh)
            mesh_name = mesh_spec.name
            mesh_filepath = self._model_dir / self._spec.meshdir / mesh_spec.file
            pass
        pass

    @property
    def root_link(self) -> str:
        return self._spec.worldbody.first_body().name

    def get_link_geometry(
        self, link_name: str, use_collision_mesh: bool = False
    ) -> list[Obstacle]:
        geoms: list[Obstacle] = []

        if body_spec := self._spec.body(link_name):
            for geom_spec in body_spec.geoms:
                assert isinstance(geom_spec, mj.MjsGeom)
                obstacle: Obstacle | None = None
                pose = Pose.from_numpy(geom_spec.pos, geom_spec.quat)
                match geom_spec.type:
                    case mj.mjtGeom.mjGEOM_BOX:
                        # TODO(wilbert): Obstacle struct internally uses 7d pos+quat, not Pose T_T
                        # although the urdf parser seems to actually use Pose instead, so the types
                        # might be wrong
                        obstacle = Cuboid(
                            name=geom_spec.name,
                            pose=pose,  # type: ignore
                            dims=(2.0 * geom_spec.size).tolist(),
                        )
                    case mj.mjtGeom.mjGEOM_SPHERE:
                        obstacle = Sphere(
                            name=geom_spec.name,
                            pose=pose,  # type: ignore
                            radius=geom_spec.size[0].item(),
                        )
                    case mj.mjtGeom.mjGEOM_CYLINDER:
                        obstacle = Cylinder(
                            name=geom_spec.name,
                            pose=pose,  # type: ignore
                            radius=geom_spec.size[0].item(),
                            height=2.0 * geom_spec.size[1].item(),
                        )
                    case mj.mjtGeom.mjGEOM_CAPSULE:
                        obstacle = Capsule(
                            name=geom_spec.name,
                            pose=pose,  # type: ignore
                            radius=geom_spec.size[0].item(),
                            base=[0, 0, -geom_spec.size[1].item()],
                            tip=[0, 0, geom_spec.size[1].item()],
                        )
                    case mj.mjtGeom.mjGEOM_MESH:
                        pass

                if obstacle:
                    geoms.append(obstacle)

        return geoms

    def get_link_mesh(self, link_name: str, use_collision_mesh: bool = False) -> CuroboMesh | None:
        pass

    def get_link_parameters(self, link_name: str, base: bool = False) -> LinkParams:
        link_params = {}

        return LinkParams(**link_params)
