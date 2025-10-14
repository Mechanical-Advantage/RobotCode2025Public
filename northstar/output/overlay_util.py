# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import cv2
import numpy
from config.config import ConfigStore
from vision_types import FiducialImageObservation, FiducialPoseObservation, ObjDetectObservation


def overlay_image_observation(image: cv2.Mat, observation: FiducialImageObservation) -> None:
    cv2.aruco.drawDetectedMarkers(image, numpy.array([observation.corners]), numpy.array([observation.tag_id]))


def overlay_pose_observation(image: cv2.Mat, config_store: ConfigStore, observation: FiducialPoseObservation) -> None:
    cv2.drawFrameAxes(
        image,
        config_store.local_config.camera_matrix,
        config_store.local_config.distortion_coefficients,
        observation.rvec_0,
        observation.tvec_0,
        config_store.remote_config.fiducial_size_m / 2,
    )
    cv2.drawFrameAxes(
        image,
        config_store.local_config.camera_matrix,
        config_store.local_config.distortion_coefficients,
        observation.rvec_1,
        observation.tvec_1,
        config_store.remote_config.fiducial_size_m / 2,
    )


def overlay_obj_detect_observation(image: cv2.Mat, observation: ObjDetectObservation) -> None:
    cv2.rectangle(
        image,
        (int(observation.corner_pixels[0][0]), int(observation.corner_pixels[0][1])),
        (int(observation.corner_pixels[3][0]), int(observation.corner_pixels[3][1])),
        (0, 0, 225),
        2,
    )
    cv2.putText(
        image,
        str(observation.obj_class) + " (" + str(round(observation.confidence * 100)) + "%)",
        (int(observation.corner_pixels[0][0]), int(observation.corner_pixels[0][1] - 5)),
        cv2.FONT_HERSHEY_PLAIN,
        2,
        (0, 0, 225),
        2,
    )
