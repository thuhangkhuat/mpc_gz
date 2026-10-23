"""Write received camera frames and their original simulation timestamps."""

import json
import math
from pathlib import Path

import cv2


class FrameVideo:
    def __init__(self, path, fps):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Video FPS must be finite and positive")
        self.path = Path(path)
        self.fps = fps
        self.writer = None
        self.timestamps = None
        self.size = None
        self.frames = 0
        self.closed = False

    def write(self, frame, stamp_ns):
        if self.closed:
            raise RuntimeError("Video is closed")
        size = (frame.shape[1], frame.shape[0])
        if self.writer is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists():
                raise FileExistsError(self.path)
            self.timestamps = self.path.with_suffix(".timestamps.jsonl").open("x")
            self.writer = cv2.VideoWriter(
                str(self.path), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, size
            )
            if not self.writer.isOpened():
                self.close()
                raise RuntimeError(f"Could not open MJPEG video writer: {self.path}")
            self.size = size
        if size != self.size:
            raise ValueError(f"Camera resolution changed from {self.size} to {size}")
        self.writer.write(frame)
        self.timestamps.write(json.dumps({"frame": self.frames, "stamp_ns": stamp_ns}) + "\n")
        self.frames += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.timestamps is not None:
            self.timestamps.close()
            self.timestamps = None
        self.closed = True
