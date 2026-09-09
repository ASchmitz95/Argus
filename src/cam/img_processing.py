import numpy as np
import numpy.typing as npt
import cv2

def detect_color(frame: npt.NDArray[np.uint8], target_coler: tuple[int, int, int], tol: float) -> npt.NDArray[np.intp]:
    """ Return (x, y) coordinates of pixels similar to an RGB target color.

    Args:
        frame: BGR image with shape (height, width, 3)
        target_color: RGB color with channel values from 0 to 255
        tol: Maximum allowd cosine distance, larger values accept more variation

    Return:
        Integer array with shape (N, 2), containing matching coordinates.
        Pixels with a color-vector lenght below 100 are excluded.    
    """

    # RGB -> BGR
    target = np.array(target_coler[::-1], dtype=np.float32)
    img = frame.astype(np.float32)

    # Length of color vector
    img_length = np.linalg.norm(img, axis=2)
    target_length = np.linalg.norm(target)

    # Similarity of color direction
    dot = np.sum(img * target, axis=2)

    similarity = dot / (np.maximum(img_length, 1) * max(target_length, 1))
    diff = 1 - similarity
    mask = (diff < tol) & (img_length >= 100)

    pixels = np.argwhere(mask)[:, ::-1]

    return pixels