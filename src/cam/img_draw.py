import cv2
import numpy as np
import numpy.typing as npt


def draw_obj(
    frame: npt.NDArray[np.uint8],
    objects: npt.NDArray[np.int32],
    color = (255, 0, 0)
) -> npt.NDArray[np.uint8]:
    """Draw bounding boxes and center points on a BGR image.

    Args:
        frame: BGR image with shape (height, width, 3).
            Modified in place.
        objects: Array with shape (N, 4), containing
            (center_x, center_y, width, height) for each object.
        color: target BGR color.

    Returns:
        The same image array with bounding boxes and center points drawn.
    """
    for x, y, w, h in objects:
        left = int(x - w // 2)
        top = int(y - h // 2)

        cv2.rectangle(
            frame,
            (left, top),
            (left + int(w) - 1, top + int(h) - 1),
            color,
            2,
        )

        cv2.circle(frame, (int(x), int(y)), 5, color, -1)

    return frame