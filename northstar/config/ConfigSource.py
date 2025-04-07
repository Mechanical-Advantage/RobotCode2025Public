# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json

import cv2
import ntcore
import numpy
from config.config import ConfigStore, RemoteConfig


class ConfigSource:
    def update(self, config_store: ConfigStore) -> None:
        raise NotImplementedError


class FileConfigSource(ConfigSource):
    def __init__(self, config_filename: str, calibration_filename: str) -> None:
        self._config_filename = config_filename
        self._calibration_filename = calibration_filename
        pass

    def update(self, config_store: ConfigStore) -> None:
        # Get config
        with open(self._config_filename, "r") as config_file:
            config_data = json.loads(config_file.read())
            config_store.local_config.device_id = config_data["device_id"]
            config_store.local_config.server_ip = config_data["server_ip"]
            config_store.local_config.apriltags_stream_port = config_data["apriltags_stream_port"]
            config_store.local_config.objdetect_stream_port = config_data["objdetect_stream_port"]
            config_store.local_config.capture_impl = config_data["capture_impl"]
            config_store.local_config.obj_detect_model = config_data["obj_detect_model"]
            config_store.local_config.obj_detect_max_fps = config_data["obj_detect_max_fps"]
            config_store.local_config.apriltags_enable = config_data["apriltags_enable"]
            config_store.local_config.objdetect_enable = config_data["objdetect_enable"]
            config_store.local_config.video_folder = config_data["video_folder"]

        # Get calibration
        calibration_store = cv2.FileStorage(self._calibration_filename, cv2.FILE_STORAGE_READ)
        camera_matrix = calibration_store.getNode("camera_matrix").mat()
        distortion_coefficients = calibration_store.getNode("distortion_coefficients").mat()
        calibration_store.release()
        if type(camera_matrix) == numpy.ndarray and type(distortion_coefficients) == numpy.ndarray:
            config_store.local_config.camera_matrix = camera_matrix
            config_store.local_config.distortion_coefficients = distortion_coefficients
            config_store.local_config.has_calibration = True


class NTConfigSource(ConfigSource):
    _init_complete: bool = False
    _camera_id_sub: ntcore.StringSubscriber
    _camera_resolution_width_sub: ntcore.IntegerSubscriber
    _camera_resolution_height_sub: ntcore.IntegerSubscriber
    _camera_auto_exposure_sub: ntcore.IntegerSubscriber
    _camera_exposure_sub: ntcore.IntegerSubscriber
    _camera_gain_sub: ntcore.DoubleSubscriber
    _camera_denoise_sub: ntcore.DoubleSubscriber
    _fiducial_size_m_sub: ntcore.DoubleSubscriber
    _tag_layout_sub: ntcore.DoubleSubscriber
    _is_recording_sub: ntcore.BooleanSubscriber
    _timestamp_sub: ntcore.IntegerSubscriber
    _event_name_sub: ntcore.StringSubscriber
    _match_type_sub: ntcore.IntegerSubscriber
    _match_number_sub: ntcore.IntegerSubscriber

    def update(self, config_store: ConfigStore) -> None:
        # Initialize subscribers on first call
        if not self._init_complete:
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + config_store.local_config.device_id + "/config"
            )
            self._camera_id_sub = nt_table.getStringTopic("camera_id").subscribe(RemoteConfig.camera_id)
            self._camera_resolution_width_sub = nt_table.getIntegerTopic("camera_resolution_width").subscribe(
                RemoteConfig.camera_resolution_width
            )
            self._camera_resolution_height_sub = nt_table.getIntegerTopic("camera_resolution_height").subscribe(
                RemoteConfig.camera_resolution_height
            )
            self._camera_auto_exposure_sub = nt_table.getIntegerTopic("camera_auto_exposure").subscribe(
                RemoteConfig.camera_auto_exposure
            )
            self._camera_exposure_sub = nt_table.getIntegerTopic("camera_exposure").subscribe(
                RemoteConfig.camera_exposure
            )
            self._camera_gain_sub = nt_table.getDoubleTopic("camera_gain").subscribe(RemoteConfig.camera_gain)
            self._camera_denoise_sub = nt_table.getDoubleTopic("camera_denoise").subscribe(RemoteConfig.camera_denoise)
            self._fiducial_size_m_sub = nt_table.getDoubleTopic("fiducial_size_m").subscribe(
                RemoteConfig.fiducial_size_m
            )
            self._tag_layout_sub = nt_table.getStringTopic("tag_layout").subscribe("")
            self._is_recording_sub = nt_table.getBooleanTopic("is_recording").subscribe(False)
            self._timestamp_sub = nt_table.getIntegerTopic("timestamp").subscribe(0)
            self._event_name_sub = nt_table.getStringTopic("event_name").subscribe("")
            self._match_type_sub = nt_table.getIntegerTopic("match_type").subscribe(0)
            self._match_number_sub = nt_table.getIntegerTopic("match_number").subscribe(0)
            self._init_complete = True

        # Read config data
        config_store.remote_config.camera_id = self._camera_id_sub.get()
        config_store.remote_config.camera_resolution_width = self._camera_resolution_width_sub.get()
        config_store.remote_config.camera_resolution_height = self._camera_resolution_height_sub.get()
        config_store.remote_config.camera_auto_exposure = self._camera_auto_exposure_sub.get()
        config_store.remote_config.camera_exposure = self._camera_exposure_sub.get()
        config_store.remote_config.camera_gain = self._camera_gain_sub.get()
        config_store.remote_config.camera_denoise = self._camera_denoise_sub.get()
        config_store.remote_config.fiducial_size_m = self._fiducial_size_m_sub.get()
        try:
            config_store.remote_config.tag_layout = json.loads(self._tag_layout_sub.get())
        except:
            config_store.remote_config.tag_layout = None
            pass
        config_store.remote_config.is_recording = self._is_recording_sub.get()
        config_store.remote_config.timestamp = self._timestamp_sub.get()
        config_store.remote_config.event_name = self._event_name_sub.get()
        config_store.remote_config.match_type = self._match_type_sub.get()
        config_store.remote_config.match_number = self._match_number_sub.get()
