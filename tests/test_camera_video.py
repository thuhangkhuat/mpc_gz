"""Verify actual encoded videos, timestamps, and recording failure behavior."""

import json

import cv2
import numpy as np
import pytest

from solo_mpc.core.video import FrameVideo


def test_video_round_trip_preserves_frames_fps_and_timestamps(tmp_path):
    path = tmp_path / "cameras" / "alpha.avi"
    video = FrameVideo(path, 8.0)
    assert not path.exists()
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 255)]
    stamps = [1_000_000_000, 1_125_000_000, 1_500_000_000]
    try:
        for color, stamp in zip(colors, stamps):
            video.write(np.full((160, 160, 3), color, dtype=np.uint8), stamp)
    finally:
        video.close()
    video.close()
    capture = cv2.VideoCapture(str(path))
    try:
        assert capture.isOpened()
        assert capture.get(cv2.CAP_PROP_FPS) == pytest.approx(8.0)
        for color in colors:
            ok, frame = capture.read()
            assert ok and frame.shape == (160, 160, 3)
            assert np.allclose(frame[80, 80], color, atol=5)
        assert not capture.read()[0]
    finally:
        capture.release()
    assert [json.loads(row) for row in path.with_suffix(".timestamps.jsonl").read_text().splitlines()] == [
        {"frame": i, "stamp_ns": stamp} for i, stamp in enumerate(stamps)
    ]


def test_no_frames_creates_no_files_and_existing_video_is_preserved(tmp_path):
    path = tmp_path / "alpha.avi"
    video = FrameVideo(path, 8.0)
    video.close()
    assert not list(tmp_path.iterdir())
    path.write_bytes(b"existing recording")
    with pytest.raises(FileExistsError):
        FrameVideo(path, 8.0).write(np.zeros((160, 160, 3), np.uint8), 0)
    assert path.read_bytes() == b"existing recording"


def test_missing_codec_fails_clearly_and_releases_writer(tmp_path, monkeypatch):
    from types import SimpleNamespace

    released = []
    monkeypatch.setattr(cv2, "VideoWriter", lambda *args: SimpleNamespace(
        isOpened=lambda: False, release=lambda: released.append(True),
    ))
    video = FrameVideo(tmp_path / "alpha.avi", 8.0)
    with pytest.raises(RuntimeError, match="Could not open MJPEG"):
        video.write(np.zeros((160, 160, 3), np.uint8), 0)
    assert released == [True]
    assert video.timestamps is None


@pytest.mark.parametrize("fps", [0, -1, float("nan"), float("inf")])
def test_invalid_fps_is_rejected(tmp_path, fps):
    with pytest.raises(ValueError, match="FPS"):
        FrameVideo(tmp_path / "alpha.avi", fps)
