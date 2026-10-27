"""The demo's screen: the annotated camera picture, and a small web server for the dashboard page.

The page is plain HTML served from this process (http://localhost:8080): the camera and the map arrive as
MJPEG streams, the state as JSON, and the buttons POST back. A browser is the one display that works from
inside the Docker container without any X11 setup, and it goes full-screen on a projector.
It listens on localhost only by default: the page can START the robot, so it is not offered to the whole Wi-Fi.
"""
from __future__ import annotations

import json
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

from .colours import colour_rgb
from .vision import Blob, Camera, FloorScan

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _label(img: np.ndarray, text: str, x: int, y: int, colour, scale: float = 0.5) -> None:
    (w, h), _ = cv2.getTextSize(text, FONT, scale, 1)
    x = max(2, min(x, img.shape[1] - w - 6))
    y = max(h + 6, min(y, img.shape[0] - 4))
    cv2.rectangle(img, (x - 2, y - h - 5), (x + w + 4, y + 3), (20, 20, 20), -1)
    cv2.putText(img, text, (x + 1, y - 2), FONT, scale, colour, 1, cv2.LINE_AA)


def annotate(bgr: np.ndarray, cam: Camera, blobs: Sequence[Blob], labels: Sequence[Optional[dict]],
             scan: Optional[FloorScan], blocked_range: float, half_width: float, countdown: Optional[float],
             show_rejected: bool = True) -> np.ndarray:
    """Draw what the robot understood on the picture. labels[i] goes with blobs[i]: None for a rejected or
    unmatched blob, else {'name', 'status'}."""
    img = bgr.copy()
    h0 = int(cam.horizon_row)
    for x in range(0, cam.width, 16):                            # the horizon the distance estimates assume
        cv2.line(img, (x, h0), (x + 8, h0), (200, 200, 120), 1)
    if scan is not None:
        near = np.isfinite(scan.x) & (scan.x <= blocked_range) & (np.abs(scan.y) <= half_width)
        for i, (u, r) in enumerate(zip(scan.u, scan.row)):
            if np.isfinite(r):
                colour = (40, 40, 255) if near[i] else (0, 165, 255)
                cv2.line(img, (int(u) - 9, int(r)), (int(u) + 9, int(r)), colour, 3 if near[i] else 2, cv2.LINE_AA)
    for blob, label in zip(blobs, labels):
        p0, p1 = (blob.x, blob.y), (blob.x + blob.w, blob.y + blob.h)
        if blob.why and label is None:
            if show_rejected:
                cv2.rectangle(img, p0, p1, (130, 130, 130), 1)
                _label(img, blob.why, blob.x, blob.y + 14, (170, 170, 170), 0.4)
            continue
        r, g, b = colour_rgb(blob.hue)
        colour = (b, g, r) if blob.hue is not None else (235, 235, 235)
        status = label['status'] if label else 'new'
        cv2.rectangle(img, p0, p1, colour, 2)
        if status == 'target':
            n = max(10, min(blob.w, blob.h) // 4)
            for cx, cy, sx, sy in ((p0[0], p0[1], 1, 1), (p1[0], p0[1], -1, 1), (p0[0], p1[1], 1, -1), (p1[0], p1[1], -1, -1)):
                cv2.line(img, (cx - 4 * sx, cy - 4 * sy), (cx + n * sx, cy - 4 * sy), (255, 255, 255), 3, cv2.LINE_AA)
                cv2.line(img, (cx - 4 * sx, cy - 4 * sy), (cx - 4 * sx, cy + n * sy), (255, 255, 255), 3, cv2.LINE_AA)
        dist = f' {100.0 * blob.area / (cam.width * cam.height):.0f}%'      # how much of the picture it covers
        text = f"{label['name'] if label else blob.name}{dist}"
        text += {'target': '  TARGET', 'visited': '  visited', 'lost': '', 'seen': '', 'new': ' ?'}[status]
        _label(img, text, blob.x, blob.y + blob.h + 20 if blob.y < 30 else blob.y - 6, (255, 255, 255))
    if countdown is not None:
        text = f'{math.ceil(countdown):d}'
        (w, h), _ = cv2.getTextSize(text, FONT, 5.0, 10)
        org = ((cam.width - w) // 2, (cam.height + h) // 2)
        cv2.putText(img, text, org, FONT, 5.0, (0, 0, 0), 18, cv2.LINE_AA)
        cv2.putText(img, text, org, FONT, 5.0, (255, 255, 255), 8, cv2.LINE_AA)
    return img


def placeholder(text: str, width: int = 640, height: int = 480) -> bytes:
    img = np.full((height, width, 3), (28, 22, 18), np.uint8)
    (w, _), _ = cv2.getTextSize(text, FONT, 0.7, 1)
    cv2.putText(img, text, ((width - w) // 2, height // 2), FONT, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
    return cv2.imencode('.jpg', img)[1].tobytes()


class Stream:
    """The latest JPEG of one picture, and a way to wait for the next."""

    def __init__(self, first: bytes):
        self._cond, self._jpeg, self._n = threading.Condition(), first, 0

    def put(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg, self._n = jpeg, self._n + 1
            self._cond.notify_all()

    def wait(self, seen: int, timeout: float = 1.0):
        with self._cond:
            self._cond.wait_for(lambda: self._n != seen, timeout)
            return self._n, self._jpeg


class Dashboard:
    """Serves the page. The node writes `camera`, `map` and `state`; the page's button presses arrive in `commands`."""

    def __init__(self, host: str, port: int):
        self.camera = Stream(placeholder('waiting for the camera ...'))
        self.map = Stream(placeholder('no map yet', 560, 560))
        self.state = '{}'
        self.commands: 'queue.Queue[dict]' = queue.Queue()
        page = (Path(__file__).parent / 'index.html').read_bytes()
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):                        # silence the per-request log
                pass

            def _send(self, code: int, kind: str, body: bytes) -> None:
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _mjpeg(self, stream: Stream) -> None:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                seen = -1
                try:
                    while not dash.closing.is_set():
                        seen, jpeg = stream.wait(seen)
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                path = self.path.split('?')[0]
                if path == '/':
                    self._send(200, 'text/html; charset=utf-8', page)
                elif path == '/state':
                    self._send(200, 'application/json', dash.state.encode())
                elif path == '/camera.mjpg':
                    self._mjpeg(dash.camera)
                elif path == '/map.mjpg':
                    self._mjpeg(dash.map)
                elif path == '/camera.jpg':
                    self._send(200, 'image/jpeg', dash.camera.wait(-1, 0.0)[1])
                elif path == '/map.jpg':
                    self._send(200, 'image/jpeg', dash.map.wait(-1, 0.0)[1])
                else:
                    self._send(404, 'text/plain', b'not found')

            def do_POST(self):
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                    if not isinstance(body, dict):
                        raise ValueError
                except (ValueError, json.JSONDecodeError):
                    return self._send(400, 'text/plain', b'bad request')
                dash.commands.put(body)
                self._send(200, 'application/json', b'{}')

        self.closing = threading.Event()
        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self) -> None:
        self.closing.set()
        self.server.shutdown()
        self.server.server_close()
