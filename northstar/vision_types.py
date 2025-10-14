# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from dataclasses import dataclass
from typing import List, Union

import numpy
import numpy.typing
from wpimath.geometry import *


@dataclass(frozen=True)
class FiducialImageObservation:
    tag_id: int
    corners: numpy.typing.NDArray[numpy.float64]


@dataclass(frozen=True)
class FiducialPoseObservation:
    tag_id: int
    pose_0: Pose3d
    error_0: float
    pose_1: Pose3d
    error_1: float


@dataclass(frozen=True)
class CameraPoseObservation:
    tag_ids: List[int]
    pose_0: Pose3d
    error_0: float
    pose_1: Union[Pose3d, None]
    error_1: Union[float, None]


@dataclass(frozen=True)
class TagAngleObservation:
    tag_id: int
    corners: numpy.typing.NDArray[numpy.float64]
    distance: float


@dataclass(frozen=True)
class ObjDetectObservation:
    obj_class: int
    confidence: float
    corner_angles: numpy.typing.NDArray[numpy.float64]
    corner_pixels: numpy.typing.NDArray[numpy.float64]
