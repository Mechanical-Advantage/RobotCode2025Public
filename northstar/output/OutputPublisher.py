# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
from typing import List, Union

import ntcore
from config.config import ConfigStore
from vision_types import CameraPoseObservation, FiducialPoseObservation, ObjDetectObservation, TagAngleObservation


class OutputPublisher:
    def send_apriltag_fps(self, config_store: ConfigStore, timestamp: float, fps: int) -> None:
        raise NotImplementedError

    def send_apriltag_observation(
        self,
        config_store: ConfigStore,
        timestamp: float,
        observation: Union[CameraPoseObservation, None],
        tag_angles: List[TagAngleObservation],
        demo_observation: Union[FiducialPoseObservation, None],
    ) -> None:
        raise NotImplementedError

    def send_objdetect_fps(self, config_store: ConfigStore, timestamp: float, fps: int) -> None:
        raise NotImplementedError

    def send_objdetect_observation(
        self, config_store: ConfigStore, timestamp: float, observations: List[ObjDetectObservation]
    ) -> None:
        raise NotImplementedError


class NTOutputPublisher(OutputPublisher):
    _init_complete: bool = False
    _observations_pub: ntcore.DoubleArrayPublisher
    _demo_observations_pub: ntcore.DoubleArrayPublisher
    _apriltags_fps_pub: ntcore.IntegerPublisher
    _objdetect_fps_pub: ntcore.IntegerPublisher
    _objdetect_observations_pub: ntcore.DoubleArrayPublisher

    def _check_init(self, config: ConfigStore):
        # Initialize publishers on first call
        if not self._init_complete:
            self._init_complete = True
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + config.local_config.device_id + "/output"
            )
            self._observations_pub = nt_table.getDoubleArrayTopic("observations").publish(
                ntcore.PubSubOptions(periodic=0.01, sendAll=True, keepDuplicates=True)
            )
            self._demo_observations_pub = nt_table.getDoubleArrayTopic("demo_observations").publish(
                ntcore.PubSubOptions(periodic=0.01, sendAll=True, keepDuplicates=True)
            )
            self._apriltags_fps_pub = nt_table.getIntegerTopic("fps_apriltags").publish()
            self._objdetect_fps_pub = nt_table.getIntegerTopic("fps_objdetect").publish()
            self._objdetect_observations_pub = nt_table.getDoubleArrayTopic("objdetect_observations").publish(
                ntcore.PubSubOptions(periodic=0.01, sendAll=True, keepDuplicates=True)
            )

    def send_apriltag_fps(self, config_store: ConfigStore, timestamp: float, fps: int) -> None:
        self._check_init(config_store)
        self._apriltags_fps_pub.set(fps)

    def send_apriltag_observation(
        self,
        config_store: ConfigStore,
        timestamp: float,
        observation: Union[CameraPoseObservation, None],
        tag_angles: List[TagAngleObservation],
        demo_observation: Union[FiducialPoseObservation, None],
    ) -> None:
        self._check_init(config_store)

        # Send data
        observation_data: List[float] = [0]
        if observation != None:
            observation_data[0] = 1
            observation_data.append(observation.error_0)
            observation_data.append(observation.pose_0.translation().X())
            observation_data.append(observation.pose_0.translation().Y())
            observation_data.append(observation.pose_0.translation().Z())
            observation_data.append(observation.pose_0.rotation().getQuaternion().W())
            observation_data.append(observation.pose_0.rotation().getQuaternion().X())
            observation_data.append(observation.pose_0.rotation().getQuaternion().Y())
            observation_data.append(observation.pose_0.rotation().getQuaternion().Z())
            if observation.error_1 != None and observation.pose_1 != None:
                observation_data[0] = 2
                observation_data.append(observation.error_1)
                observation_data.append(observation.pose_1.translation().X())
                observation_data.append(observation.pose_1.translation().Y())
                observation_data.append(observation.pose_1.translation().Z())
                observation_data.append(observation.pose_1.rotation().getQuaternion().W())
                observation_data.append(observation.pose_1.rotation().getQuaternion().X())
                observation_data.append(observation.pose_1.rotation().getQuaternion().Y())
                observation_data.append(observation.pose_1.rotation().getQuaternion().Z())
        for tag_angle_observation in tag_angles:
            observation_data.append(tag_angle_observation.tag_id)
            for angle in tag_angle_observation.corners.ravel():
                observation_data.append(angle)
            observation_data.append(tag_angle_observation.distance)

        demo_observation_data: List[float] = []
        if demo_observation != None:
            demo_observation_data.append(demo_observation.error_0)
            demo_observation_data.append(demo_observation.pose_0.translation().X())
            demo_observation_data.append(demo_observation.pose_0.translation().Y())
            demo_observation_data.append(demo_observation.pose_0.translation().Z())
            demo_observation_data.append(demo_observation.pose_0.rotation().getQuaternion().W())
            demo_observation_data.append(demo_observation.pose_0.rotation().getQuaternion().X())
            demo_observation_data.append(demo_observation.pose_0.rotation().getQuaternion().Y())
            demo_observation_data.append(demo_observation.pose_0.rotation().getQuaternion().Z())
            demo_observation_data.append(demo_observation.error_1)
            demo_observation_data.append(demo_observation.pose_1.translation().X())
            demo_observation_data.append(demo_observation.pose_1.translation().Y())
            demo_observation_data.append(demo_observation.pose_1.translation().Z())
            demo_observation_data.append(demo_observation.pose_1.rotation().getQuaternion().W())
            demo_observation_data.append(demo_observation.pose_1.rotation().getQuaternion().X())
            demo_observation_data.append(demo_observation.pose_1.rotation().getQuaternion().Y())
            demo_observation_data.append(demo_observation.pose_1.rotation().getQuaternion().Z())

        self._observations_pub.set(observation_data, math.floor(timestamp * 1000000))
        self._demo_observations_pub.set(demo_observation_data, math.floor(timestamp * 1000000))

    def send_objdetect_fps(self, config_store: ConfigStore, timestamp: float, fps: int) -> None:
        self._check_init(config_store)
        self._objdetect_fps_pub.set(fps)

    def send_objdetect_observation(
        self, config_store: ConfigStore, timestamp: float, observations: List[ObjDetectObservation]
    ) -> None:
        self._check_init(config_store)

        observation_data: List[float] = []
        for observation in observations:
            observation_data.append(observation.obj_class)
            observation_data.append(observation.confidence)
            for angle in observation.corner_angles.ravel():
                observation_data.append(angle)

        self._objdetect_observations_pub.set(observation_data, math.floor(timestamp * 1000000))
