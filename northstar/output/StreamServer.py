# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import random
import socketserver
import string
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO
from typing import Dict

import cv2
from PIL import Image


CLIENT_COUNTS: Dict[str, int] = {}


class StreamServer:
    """Interface for outputing camera frames."""

    def start(self, port: int) -> None:
        """Starts the output stream."""
        raise NotImplementedError

    def set_frame(self, frame: cv2.Mat) -> None:
        """Sets the frame to serve."""
        raise NotImplementedError


class MjpegServer(StreamServer):
    _frame: cv2.Mat
    _has_frame: bool = False
    _uuid: str = ""

    def _make_handler(self_mjpeg, uuid: str):  # type: ignore
        class StreamingHandler(BaseHTTPRequestHandler):
            HTML = """
    <html>
        <head>
            <title>Northstar Debug</title>
            <style>
                body {
                    background-color: black;
                }

                img {
                    position: absolute;
                    left: 50%;
                    top: 50%;
                    transform: translate(-50%, -50%);
                    max-width: 100%;
                    max-height: 100%;
                }
            </style>
        </head>
        <body>
            <img src="stream.mjpg" />
        </body>
    </html>
            """

            def do_GET(self):
                global CLIENT_COUNTS
                if self.path == "/":
                    content = self.HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                elif self.path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                    self.end_headers()
                    try:
                        CLIENT_COUNTS[uuid] += 1
                        while True:
                            if not self_mjpeg._has_frame:
                                time.sleep(0.1)
                            else:
                                pil_im = Image.fromarray(self_mjpeg._frame)
                                stream = BytesIO()
                                pil_im.save(stream, format="JPEG")
                                frame_data = stream.getvalue()

                                self.wfile.write(b"--FRAME\r\n")
                                self.send_header("Content-Type", "image/jpeg")
                                self.send_header("Content-Length", str(len(frame_data)))
                                self.end_headers()
                                self.wfile.write(frame_data)
                                self.wfile.write(b"\r\n")
                    except Exception as e:
                        print("Removed streaming client %s: %s", self.client_address, str(e))
                    finally:
                        CLIENT_COUNTS[uuid] -= 1
                else:
                    self.send_error(404)
                    self.end_headers()

        return StreamingHandler

    class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    def _run(self, port: int) -> None:
        self._uuid = "".join(random.choice(string.ascii_lowercase) for i in range(12))
        CLIENT_COUNTS[self._uuid] = 0
        server = self.StreamingServer(("", port), self._make_handler(self._uuid))
        server.serve_forever()

    def start(self, port: int) -> None:
        threading.Thread(target=self._run, daemon=True, args=(port,)).start()

    def set_frame(self, frame: cv2.Mat) -> None:
        self._frame = frame
        self._has_frame = True

    def get_client_count(self) -> int:
        if len(self._uuid) > 0:
            return CLIENT_COUNTS[self._uuid]
        else:
            return 0
