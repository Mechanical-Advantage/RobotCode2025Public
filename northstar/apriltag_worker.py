# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import queue
from typing import List, Tuple, Union

import cv2
from config.config import ConfigStore
from output.overlay_util import overlay_image_observation
from output.StreamServer import MjpegServer
from pipeline.CameraPoseEstimator import MultiTargetCameraPoseEstimator
from pipeline.FiducialDetector import ArucoFiducialDetector
from pipeline.PoseEstimator import SquareTargetPoseEstimator
from pipeline.TagAngleCalculator import CameraMatrixTagAngleCalculator
from vision_types import CameraPoseObservation, FiducialImageObservation, FiducialPoseObservation, TagAngleObservation

DEMO_ID = 42


def apriltag_worker(
    q_in: queue.Queue[Tuple[float, cv2.Mat, ConfigStore]],
    q_out: queue.Queue[
        Tuple[
            float,
            List[FiducialImageObservation],
            Union[CameraPoseObservation, None],
            List[TagAngleObservation],
            Union[FiducialPoseObservation, None],
        ]
    ],
    server_port: int,
):
    fiducial_detector = ArucoFiducialDetector(cv2.aruco.DICT_APRILTAG_36h11)
    camera_pose_estimator = MultiTargetCameraPoseEstimator()
    tag_angle_calculator = CameraMatrixTagAngleCalculator()
    tag_pose_estimator = SquareTargetPoseEstimator()
    stream_server = MjpegServer()
    stream_server.start(server_port)

    while True:
        sample = q_in.get()
        timestamp: float = sample[0]
        image: cv2.Mat = sample[1]
        config: ConfigStore = sample[2]

        image_observations = fiducial_detector.detect_fiducials(image, config)
        camera_pose_observation = camera_pose_estimator.solve_camera_pose(
            [x for x in image_observations if x.tag_id != DEMO_ID], config
        )
        tag_angle_observations = [
            tag_angle_calculator.calc_tag_angles(x, config) for x in image_observations if x.tag_id != DEMO_ID
        ]
        tag_angle_observations = [x for x in tag_angle_observations if x != None]
        demo_image_observations = [x for x in image_observations if x.tag_id == DEMO_ID]
        demo_pose_observation: Union[FiducialPoseObservation, None] = None
        if len(demo_image_observations) > 0:
            demo_pose_observation = tag_pose_estimator.solve_fiducial_pose(demo_image_observations[0], config)

        q_out.put(
            (timestamp, image_observations, camera_pose_observation, tag_angle_observations, demo_pose_observation)
        )
        if stream_server.get_client_count() > 0:
            image = image.copy()
            [overlay_image_observation(image, x) for x in image_observations]
            stream_server.set_frame(image)
