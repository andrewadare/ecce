import cv2
import numpy as np


# Parse aruco module for dictionary names and enumerations
ARUCO_CODES = {k: v for k, v in cv2.aruco.__dict__.items() if k.startswith("DICT_")}


def create_tag(codebook: str, tag_id: int, tag_width: int, border: int) -> np.ndarray:
    """Create an Aruco or April tag image using the cv2.aruco module.

    Usage
    =====
    Tag codebook options are discoverable in cv2.aruco.__dict__ as "DICT_*".

    To make a 400x400 px image of a 300x300 April 16h5 tag with id 0 and a
    50 px white border, do

    tag = create_tag("DICT_APRILTAG_16h5", 0, 300, 50)
    """

    if codebook not in ARUCO_CODES:
        raise LookupError(
            f"Tag dictionary '{codebook}' not found. Available options:\n"
            f"{' '.join(ARUCO_CODES.keys())}"
        )

    code = cv2.aruco.getPredefinedDictionary(ARUCO_CODES[codebook])
    tag = cv2.aruco.generateImageMarker(code, tag_id, tag_width)

    w = tag_width + 2 * border
    im = np.full((w, w), 255, dtype=np.uint8)
    im[border:-border, border:-border] = tag

    return cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
