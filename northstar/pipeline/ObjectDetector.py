# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
from typing import List, Union

import coremltools
import cv2
import numpy as np
from config.config import ConfigStore
from PIL import Image
from vision_types import ObjDetectObservation


class ObjectDetector:
    def __init__(self) -> None:
        raise NotImplementedError

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        raise NotImplementedError


class CoreMLObjectDetector(ObjectDetector):
    _model: Union[coremltools.models.MLModel, None] = None

    def __init__(self) -> None:
        pass

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        # Load CoreML model
        if self._model == None:
            print("Loading object detection model")
            self._model = coremltools.models.MLModel(config.local_config.obj_detect_model)
            print("Finished loading object detection model")

        # Create scaled frame for model
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        image_scaled = np.zeros((640, 640, 3), dtype=np.uint8)
        scaled_height = int(640 / (image.shape[1] / image.shape[0]))
        bar_height = int((640 - scaled_height) / 2)
        image_scaled[bar_height : bar_height + scaled_height, 0:640] = cv2.resize(image, (640, scaled_height))

        # Run CoreML model
        image_coreml = Image.fromarray(image_scaled)
        prediction = self._model.predict({"image": image_coreml})

        observations: List[ObjDetectObservation] = []
        for coordinates, confidence in zip(prediction["coordinates"], prediction["confidence"]):
            obj_class = max(range(len(confidence)), key=confidence.__getitem__)
            confidence = float(confidence[obj_class])
            x = coordinates[0] * image.shape[1]
            y = ((coordinates[1] * 640 - bar_height) / scaled_height) * image.shape[0]
            width = coordinates[2] * image.shape[1]
            height = coordinates[3] / (scaled_height / 640) * image.shape[0]

            corners = np.array(
                [
                    [x - width / 2, y - height / 2],
                    [x + width / 2, y - height / 2],
                    [x - width / 2, y + height / 2],
                    [x + width / 2, y + height / 2],
                ]
            )
            corners_undistorted = cv2.undistortPoints(
                corners,
                config.local_config.camera_matrix,
                config.local_config.distortion_coefficients,
                None,
                config.local_config.camera_matrix,
            )

            corner_angles = np.zeros((4, 2))
            for index, corner in enumerate(corners_undistorted):
                vec = np.linalg.inv(config.local_config.camera_matrix).dot(np.array([corner[0][0], corner[0][1], 1]).T)
                corner_angles[index][0] = math.atan(vec[0])
                corner_angles[index][1] = math.atan(vec[1])

            observations.append(ObjDetectObservation(obj_class, confidence, corner_angles, corners))

        return observations
