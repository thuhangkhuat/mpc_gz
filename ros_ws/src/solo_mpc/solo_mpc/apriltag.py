"""AprilTag rendering and detection through OpenCV's aruco module.

Supports the OpenCV 4.6 API in Ubuntu's python3-opencv and the 4.7+ API.
Images and detections use the standard tag36h11 codes, so a printed tag from
the AprilTag project is detected the same way on hardware.
"""

import numpy as np

FAMILIES = {"tag36h11": "DICT_APRILTAG_36h11"}
CELLS = 8  # 6 x 6 data bits plus a one-cell black border


def _aruco():
    import cv2

    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV lacks the aruco module; install opencv with contrib modules")
    return cv2, cv2.aruco


def dictionary(family):
    cv2, aruco = _aruco()
    if family not in FAMILIES:
        raise ValueError(f"Unsupported tag family {family!r}; use one of {sorted(FAMILIES)}")
    return aruco.getPredefinedDictionary(getattr(aruco, FAMILIES[family]))


def tag_image(family, tag_id, cell_px=24, quiet_cells=1):
    """Grayscale tag with a white quiet zone; black square is CELLS * cell_px wide."""
    cv2, aruco = _aruco()
    side = CELLS * cell_px
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary(family), tag_id, side)
    else:
        marker = aruco.drawMarker(dictionary(family), tag_id, side)
    pad = quiet_cells * cell_px
    return cv2.copyMakeBorder(marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def _detect(gray, family):
    cv2, aruco = _aruco()
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary(family), aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, dictionary(family),
                                              parameters=aruco.DetectorParameters_create())
    return corners, ids


def detect(gray, family, tag_id):
    """Pixel centre and 4x2 corners of tag_id, or None.

    A rendered texture may appear mirrored depending on UV mapping, and a
    mirrored code decodes as nothing; a second pass on the flipped image
    recovers it, mapped back to original pixel coordinates.
    """
    width = gray.shape[1]
    for mirrored in (False, True):
        image = gray[:, ::-1].copy() if mirrored else gray
        corners, ids = _detect(image, family)
        if ids is None:
            continue
        for found, quad in zip(np.asarray(ids).reshape(-1), corners):
            if int(found) != tag_id:
                continue
            quad = np.asarray(quad, float).reshape(4, 2)
            if mirrored:
                quad[:, 0] = width - 1 - quad[:, 0]
            return quad.mean(axis=0), quad
    return None
