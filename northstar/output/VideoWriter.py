# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from typing import List
import cv2
from datetime import datetime
import subprocess
import queue
import threading

from config.config import ConfigStore
from output.overlay_util import overlay_image_observation, overlay_obj_detect_observation
from vision_types import FiducialImageObservation, ObjDetectObservation

FRAMERATE = 25


class VideoWriter:
    def __init__(self) -> None:
        raise NotImplementedError

    def start(self, config: ConfigStore, is_gray: bool) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def write_frame(self, timestamp: float, frame: cv2.Mat) -> None:
        raise NotImplementedError


class FFmpegVideoWriter(VideoWriter):
    def __init__(self) -> None:
        pass

    def start(self, config: ConfigStore, is_gray: bool) -> None:
        match_prefixes = ["", "p", "q", "e"]
        filename_base = (
            config.local_config.video_folder
            + config.local_config.device_id
            + "_"
            + datetime.fromtimestamp(config.remote_config.timestamp).strftime("%Y%m%d_%H%M%S")
        )
        filename_match = ""
        if len(config.remote_config.event_name) > 0:
            filename_match += "_" + config.remote_config.event_name.lower()
        if config.remote_config.match_type != 0 and config.remote_config.match_number > 0:
            filename_match += (
                "_" + match_prefixes[config.remote_config.match_type] + str(config.remote_config.match_number)
            )
        filename = filename_base + filename_match + ".mkv"
        filename_raw = filename_base + "_raw" + filename_match + ".mkv"

        ffmpeg_args_base = [
            "ffmpeg",
            "-y",
            "-s",
            str(config.remote_config.camera_resolution_width)
            + "x"
            + str(config.remote_config.camera_resolution_height),
            "-pixel_format",
            "gray" if is_gray else "rgb24",
            "-r",
            str(FRAMERATE),
            "-re",
            "-f",
            "rawvideo",
            "-i",
            "pipe:",
            "-c:v",
            "hevc_videotoolbox",
            "-q:v",
            "50",
            "-pix_fmt",
            "yuv420p",
            "-vf",
            "setpts=PTS-STARTPTS",
        ]
        self._ffmpeg = subprocess.Popen(ffmpeg_args_base + [filename], stdin=subprocess.PIPE)
        self._ffmpeg_raw = subprocess.Popen(ffmpeg_args_base + [filename_raw], stdin=subprocess.PIPE)

        self._running = True
        self._queue = queue.Queue(maxsize=1)
        self._queue_raw = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._frame_thread, args=(self._queue, False), daemon=True)
        self._thread_raw = threading.Thread(target=self._frame_thread, args=(self._queue, True), daemon=True)
        self._thread.start()
        self._thread_raw.start()

    def stop(self) -> None:
        self._running = False
        self._thread.join()
        self._thread_raw.join()
        self._ffmpeg.kill()
        self._ffmpeg_raw.kill()

    def write_frame(
        self,
        timestamp: float,
        frame: cv2.Mat,
        image_observations: List[FiducialImageObservation],
        obj_detect_observations: List[ObjDetectObservation],
    ) -> None:
        try:
            self._queue.put((frame, image_observations, obj_detect_observations), block=False)
            self._queue_raw.put((frame, [], []), block=False)
        except:
            pass

    def _frame_thread(self, q_in: queue.Queue[cv2.Mat], is_raw: bool) -> None:
        while self._running:
            try:
                frame, image_observations, obj_detect_observations = q_in.get(timeout=0.1)
            except:
                pass
            else:
                if not is_raw:
                    frame = frame.copy()
                    [overlay_image_observation(frame, x) for x in image_observations]
                    [overlay_obj_detect_observation(frame, x) for x in obj_detect_observations]
                ffmpeg = self._ffmpeg_raw if is_raw else self._ffmpeg
                ffmpeg.stdin.write(frame.tobytes())
