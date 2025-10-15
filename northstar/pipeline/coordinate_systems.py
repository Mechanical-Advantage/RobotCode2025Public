# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
from typing import List

import numpy
import numpy.typing
from wpimath.geometry import *


def openCvPoseToWpilib(tvec: numpy.typing.NDArray[numpy.float64], rvec: numpy.typing.NDArray[numpy.float64]) -> Pose3d:
    return Pose3d(
        Translation3d(tvec[2][0], -tvec[0][0], -tvec[1][0]),
        Rotation3d(
            numpy.array([rvec[2][0], -rvec[0][0], -rvec[1][0]]),
            math.sqrt(math.pow(rvec[0][0], 2) + math.pow(rvec[1][0], 2) + math.pow(rvec[2][0], 2)),
        ),
    )


def wpilibTranslationToOpenCv(translation: Translation3d) -> List[float]:
    return [-translation.Y(), -translation.Z(), translation.X()]
