# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import dataclasses
import subprocess
import sys
import time
import traceback
from typing import Tuple, Union

import AVFoundation
import cv2
import numpy
from config.config import ConfigStore
from pypylon import pylon


class Capture:
    """Interface for receiving camera frames."""

    def __init__(self) -> None:
        raise NotImplementedError

    def get_frame(self, config_store: ConfigStore) -> Tuple[bool, cv2.Mat]:
        """Return the next frame from the camera."""
        raise NotImplementedError

    @classmethod
    def _config_changed(cls, config_a: ConfigStore, config_b: ConfigStore) -> bool:
        if config_a == None and config_b == None:
            return False
        if config_a == None or config_b == None:
            return True

        remote_a = config_a.remote_config
        remote_b = config_b.remote_config

        return (
            remote_a.camera_id != remote_b.camera_id
            or remote_a.camera_resolution_width != remote_b.camera_resolution_width
            or remote_a.camera_resolution_height != remote_b.camera_resolution_height
            or remote_a.camera_auto_exposure != remote_b.camera_auto_exposure
            or remote_a.camera_exposure != remote_b.camera_exposure
            or remote_a.camera_gain != remote_b.camera_gain
        )


class DefaultCapture(Capture):
    """ "Read from camera with default OpenCV config."""

    def __init__(self) -> None:
        pass

    _video = None
    _last_config: ConfigStore

    def get_frame(self, config_store: ConfigStore) -> Tuple[bool, cv2.Mat]:
        if self._video != None and self._config_changed(self._last_config, config_store):
            print("Restarting capture session")
            self._video.release()
            self._video = None

        if self._video == None:
            self._video = cv2.VideoCapture(config_store.remote_config.camera_id)
            self._video.set(cv2.CAP_PROP_FRAME_WIDTH, config_store.remote_config.camera_resolution_width)
            self._video.set(cv2.CAP_PROP_FRAME_HEIGHT, config_store.remote_config.camera_resolution_height)
            self._video.set(cv2.CAP_PROP_AUTO_EXPOSURE, config_store.remote_config.camera_auto_exposure)
            self._video.set(cv2.CAP_PROP_EXPOSURE, config_store.remote_config.camera_exposure)
            self._video.set(cv2.CAP_PROP_GAIN, int(config_store.remote_config.camera_gain))

        self._last_config = config_store

        retval, image = self._video.read()
        return retval, image


class AVFoundationCapture(Capture):
    """ "Read from camera with OpenCV and AVFoundation."""

    def __init__(self) -> None:
        pass

    _video = None
    _last_config: ConfigStore

    def get_frame(self, config_store: ConfigStore) -> Tuple[bool, cv2.Mat]:
        if self._video != None and self._config_changed(self._last_config, config_store):
            print("Restarting capture session")
            self._video.release()
            self._video = None
            sys.exit(1)

        if self._video == None:
            camera_id_split = config_store.remote_config.camera_id.split(":")
            if config_store.remote_config.camera_id == "" or len(camera_id_split) != 3:
                print("No camera ID, waiting to start capture session")
            else:
                devices = list(AVFoundation.AVCaptureDevice.devicesWithMediaType_(AVFoundation.AVMediaTypeVideo))
                devices.sort(key=lambda x: x.uniqueID())
                for index, device in enumerate(devices):
                    if device.uniqueID() == config_store.remote_config.camera_id.replace(":", ""):
                        camera_location_id = camera_id_split[0]
                        camera_vendor_id = camera_id_split[1]
                        camera_product_id = camera_id_split[2]

                        subprocess.run(
                            [
                                "./ns-iokit-ctl/build/ns_iokit_ctl",
                                camera_vendor_id,
                                camera_product_id,
                                camera_location_id,
                                str(config_store.remote_config.camera_auto_exposure),
                                str(config_store.remote_config.camera_exposure),
                                str(int(config_store.remote_config.camera_gain)),
                            ],
                            check=True,
                        )

                        self._video = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
                        self._video.set(cv2.CAP_PROP_FRAME_WIDTH, config_store.remote_config.camera_resolution_width)
                        self._video.set(cv2.CAP_PROP_FRAME_HEIGHT, config_store.remote_config.camera_resolution_height)
                        break

        self._last_config = ConfigStore(
            dataclasses.replace(config_store.local_config), dataclasses.replace(config_store.remote_config)
        )

        if self._video == None:
            if config_store.remote_config.camera_id != "":
                print("Camera not found, restarting")
                sys.exit(1)
            return False, None
        else:
            retval, image = self._video.read()
            if not retval:
                print("Capture session failed, restarting")
                self._video.release()
                self._video = None  # Force reconnect
                sys.exit(1)
            return retval, image


class PylonCapture(Capture):
    """Reads from a Basler camera using pylon."""

    def __init__(self, mode: str = "", is_flipped: bool = False) -> None:
        self._mode = mode
        self._is_flipped = is_flipped

    _camera: Union[None, pylon.InstantCamera] = None
    _converter: Union[None, pylon.ImageFormatConverter] = None
    _last_config: ConfigStore

    def get_frame(self, config_store: ConfigStore) -> Tuple[bool, cv2.Mat]:
        if self._camera != None and self._config_changed(self._last_config, config_store):
            print("Config changed, stopping capture session")
            self._camera.Close()
            self._camera = None
            time.sleep(2)

        if self._camera is None:
            device_infos: list[pylon.DeviceInfo] = pylon.TlFactory.GetInstance().EnumerateDevices()
            device: Union[None, any] = None  # Native object type
            for device_info in device_infos:
                if device_info.GetSerialNumber() == config_store.remote_config.camera_id:
                    device = pylon.TlFactory.GetInstance().CreateDevice(device_info)
            if device != None:
                print("Starting capture session")
                self._camera = pylon.InstantCamera(device)
                self._camera.Open()
                self._camera.GetNodeMap().GetNode("DeviceLinkThroughputLimitMode").SetValue("On")
                max_bandwidth = int(150e6) if self._mode == "color" else int(250e6)
                self._camera.GetNodeMap().GetNode("DeviceLinkThroughputLimit").SetValue(max_bandwidth)
                self._camera.GetNodeMap().GetNode("ExposureAuto").SetValue("Off")
                self._camera.GetNodeMap().GetNode("ExposureTime").SetValue(config_store.remote_config.camera_exposure)
                self._camera.GetNodeMap().GetNode("GainAuto").SetValue("Off")
                self._camera.GetNodeMap().GetNode("Gain").SetValue(config_store.remote_config.camera_gain)

                if self._mode == "color":
                    self._converter = pylon.ImageFormatConverter()
                    self._converter.OutputPixelFormat = pylon.PixelType_RGB8packed
                    self._converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

                elif self._mode == "cropped":
                    self._camera.GetNodeMap().GetNode("Width").SetValue(1600)
                    self._camera.GetNodeMap().GetNode("Height").SetValue(1200)
                    self._camera.GetNodeMap().GetNode("OffsetX").SetValue(168)
                    self._camera.GetNodeMap().GetNode("OffsetY").SetValue(8)

                if self._is_flipped:
                    self._camera.GetNodeMap().GetNode("ReverseX").SetValue(True)
                    self._camera.GetNodeMap().GetNode("ReverseY").SetValue(True)

                self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                print("Capture session ready")

        self._last_config = ConfigStore(
            dataclasses.replace(config_store.local_config), dataclasses.replace(config_store.remote_config)
        )

        if self._camera is None:
            return False, None
        else:
            with self._camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException) as grab_result:
                if grab_result.GrabSucceeded:
                    try:
                        if self._converter == None:
                            return True, grab_result.Array
                        else:
                            return True, self._converter.Convert(grab_result).Array
                    except Exception:
                        print("Error when capturing frame:", traceback.format_exc())
                        return False, None
                else:
                    return False, None


class GStreamerCapture(Capture):
    """ "Read from camera with GStreamer."""

    def __init__(self) -> None:
        pass

    _video = None
    _last_config: ConfigStore

    def get_frame(self, config_store: ConfigStore) -> Tuple[bool, cv2.Mat]:
        if self._video != None and self._config_changed(self._last_config, config_store):
            print("Config changed, stopping capture session")
            self._video.release()
            self._video = None
            time.sleep(2)

        if self._video == None:
            if config_store.remote_config.camera_id == "":
                print("No camera ID, waiting to start capture session")
            else:
                print("Starting capture session")
                self._video = cv2.VideoCapture(
                    "v4l2src device="
                    + str(config_store.remote_config.camera_id)
                    + ' extra_controls="c,exposure_auto='
                    + str(config_store.remote_config.camera_auto_exposure)
                    + ",exposure_absolute="
                    + str(config_store.remote_config.camera_exposure)
                    + ",gain="
                    + str(int(config_store.remote_config.camera_gain))
                    + ',sharpness=0,brightness=0" ! image/jpeg,format=MJPG,width='
                    + str(config_store.remote_config.camera_resolution_width)
                    + ",height="
                    + str(config_store.remote_config.camera_resolution_height)
                    + " ! jpegdec ! video/x-raw ! appsink drop=1",
                    cv2.CAP_GSTREAMER,
                )
                print("Capture session ready")

        self._last_config = ConfigStore(
            dataclasses.replace(config_store.local_config), dataclasses.replace(config_store.remote_config)
        )

        if self._video != None:
            retval, image = self._video.read()
            if not retval:
                print("Capture session failed, restarting")
                self._video.release()
                self._video = None  # Force reconnect
                sys.exit(1)
            return retval, image
        else:
            return False, cv2.Mat(numpy.ndarray([]))


CAPTURE_IMPLS = {
    "": DefaultCapture,
    "avfoundation": AVFoundationCapture,
    "pylon": lambda: PylonCapture(),
    "pylon-flipped": lambda: PylonCapture(is_flipped=True),
    "pylon-color": lambda: PylonCapture("color"),
    "pylon-color-flipped": lambda: PylonCapture("color", is_flipped=True),
    "pylon-cropped": lambda: PylonCapture("cropped"),
    "pylon-cropped-flipped": lambda: PylonCapture("cropped", is_flipped=True),
    "gstreamer": GStreamerCapture,
}
