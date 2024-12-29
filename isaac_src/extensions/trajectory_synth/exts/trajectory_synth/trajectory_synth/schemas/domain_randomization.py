import omni.replicator.core as rep
from pydantic import BaseModel

from .constants import BOUND


class PositionDistribution(BaseModel):
    pos_offset_max: tuple[float, float, float]
    pos_offset_min: tuple[float, float, float] | None = None

    rot_offset_max: tuple[float, float, float]
    rot_offset_min: tuple[float, float, float] | None = None

    @property
    def position_distribution(self):
        return self._uniform(self.pos_offset_max, self.pos_offset_min)

    @property
    def rotation_distribution(self):
        return self._uniform(self.rot_offset_max, self.rot_offset_min)

    def _uniform(self, max_bound: BOUND, min_bound: BOUND | None):
        if min_bound is None:
            min_bound = [-n for n in max_bound]
        # Sanity check
        assert all(min_bound[i] <= max_bound[i] for i in range(3))
        return rep.distribution.uniform(min_bound, max_bound)


class DistractorDistribution(PositionDistribution):
    scale_max: tuple[float, float, float]
    scale_min: tuple[float, float, float] | None = None

    @property
    def scale_distribution(self):
        return self._uniform(self.scale_max, self.scale_min)


class CameraDistribution(PositionDistribution):
    pass


class RandomizationDistributions(BaseModel):
    # Choose how fast or slow the robot moves. This allows you to "stretch" or
    # "squish" the trajectory in time.
    min_render_fps: int = 10
    max_render_fps: int = 50

    # Choose number of materials to randomize
    min_materials: int = 1
    max_materials: int = 50

    # Choose how much to modify lighting position
    light_pos_offset: tuple[float, float, float] = (0.1, 0.1, 0.1)
    light_rot_offset: tuple[float, float, float] = (45, 45, 45)

    # Choose how to modify different prims in the scene
    distractor_params: dict[str, DistractorDistribution] = {
        "/World/GroundPlane": DistractorDistribution(
            pos_offset_max=(1.0, 1.0, 0.0),
            pos_offset_min=(-1.0, -1.0, -0.0),
            rot_offset_max=(0, 0, 360),
            rot_offset_min=(0, 0, -360),
            scale_max=(10.0, 10.0, 1.0),
            scale_min=(0.1, 0.1, 1.0),
        ),
        # Stretch and squish the ENTIRE world a little bit, so the algorithm has to
        # learn to deal with different features on the robots, and can't rely on the
        # exact straight lines and angles the robot makes.
        "/World": DistractorDistribution(
            pos_offset_max=(0, 0, 0),
            pos_offset_min=(0, 0, 0),
            rot_offset_max=(0, 0, 0),
            rot_offset_min=(0, 0, 0),
            scale_max=(1.2, 1.2, 1.2),
            scale_min=(0.8, 0.8, 0.8),
        ),
    }
    # Camera name -> randomization params
    # One is randomly chosen per full trajectory run
    camera_params: dict[str, list[CameraDistribution]] = {
        "top": [
            # Vary rotation as much as possible
            CameraDistribution(
                pos_offset_min=(0, 0, 0),
                pos_offset_max=(0, 0, 0),
                rot_offset_min=(-10.0, -6.0, -5.0),  # Validated
                rot_offset_max=(1.0, 6.0, 5.0),  # Validated
            ),
            # Vary position around within a small range
            CameraDistribution(
                pos_offset_min=(-0.08, -0.05, -0.025),  # Validated
                pos_offset_max=(0.08, 0.02, 0.25),  # Validated
                rot_offset_min=(0, 0, 0),
                rot_offset_max=(0, 0, 0),
            ),
            # Keep the Z far from the normal position, allowing for drastic rotations
            # around the Z axis
            CameraDistribution(
                pos_offset_min=(0, 0, 0.15),
                pos_offset_max=(0, 0, 0.35),
                rot_offset_min=(0.0, 0, -50),
                rot_offset_max=(0.0, 0, 50),
            ),
        ],
        "wrist": [
            # Strategy: Set rot offset to 10, and maximize the position offset while
            #           keeping the picker tip in view
            CameraDistribution(
                pos_offset_max=(0.01, 0.005, 0.015),  # Validated
                rot_offset_min=[-15, -10, -10],  # Validated
                rot_offset_max=[7, 10, 10],  # Validated
            )
        ],
    }
