# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
from typing import Union

import cv2
import numpy as np
from config.config import ConfigStore
from pipeline.PoseEstimator import SquareTargetPoseEstimator
from vision_types import FiducialImageObservation, FiducialPoseObservation, TagAngleObservation


class TagAngleCalculator:
    def __init__(self) -> None:
        raise NotImplementedError

    def calc_tag_angles(
        self, image_observation: FiducialImageObservation, config_store: ConfigStore
    ) -> Union[TagAngleObservation, None]:
        raise NotImplementedError


class CameraMatrixTagAngleCalculator(TagAngleCalculator):
    _tag_pose_estimator = SquareTargetPoseEstimator()

    def __init__(self) -> None:
        pass

    def calc_tag_angles(
        self, image_observation: FiducialImageObservation, config_store: ConfigStore
    ) -> Union[TagAngleObservation, None]:
        # Undistort corners
        corners_undistorted = cv2.undistortPoints(
            image_observation.corners,
            config_store.local_config.camera_matrix,
            config_store.local_config.distortion_coefficients,
            None,
            config_store.local_config.camera_matrix,
        )

        # Calculate angles
        corners = np.zeros((4, 2))
        for index, corner in enumerate(corners_undistorted):
            vec = np.linalg.inv(config_store.local_config.camera_matrix).dot(
                np.array([corner[0][0], corner[0][1], 1]).T
            )
            corners[index][0] = math.atan(vec[0])
            corners[index][1] = math.atan(vec[1])

        # Get distance
        pose_observation: FiducialPoseObservation = self._tag_pose_estimator.solve_fiducial_pose(
            image_observation, config_store
        )
        if pose_observation == None:
            return None
        distance: float = 0
        if pose_observation.error_0 < pose_observation.error_1:
            distance = pose_observation.pose_0.translation().norm()
        else:
            distance = pose_observation.pose_1.translation().norm()

        # Publish result
        return TagAngleObservation(image_observation.tag_id, corners, distance)
