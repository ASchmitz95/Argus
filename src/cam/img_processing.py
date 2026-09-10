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

def detect_objects(pixels: npt.NDArray[np.intp], min_area: int, gap: int = 5,) -> npt.NDArray[np.int32]:
    """Group pixels into connected regions and return their bounding boxes.

    Args:
        pixels: Non-negative (x, y) pixel coordinates with shape (N, 2).
        min_area: Minimum component area in pixels after morphological closing.
        gap: Side length of the square closing kernel. Must be positive.

    Returns:
        Integer array with shape (M, 4). Each row contains
        (center_x, center_y, width, height) of a bounding box.
        Boxes fully contained in another component's box are excluded.
    """
    if len(pixels) == 0:
        return np.empty((0, 4), dtype=np.int32)

    width = pixels[:, 0].max() + 1
    height = pixels[:, 1].max() + 1

    mask = np.zeros((height, width), dtype=np.uint8)
    mask[pixels[:, 1], pixels[:, 0]] = 255

    # Close small holes and bridge nearby regions.
    kernel = np.ones((gap, gap), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    count, _, stats, _ = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )

    components = []

    # Label 0 represents the background.
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < min_area:
            continue

        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]

        components.append((x, y, w, h))

    objects = []

    for i, (x, y, w, h) in enumerate(components):
        inside_other = False

        for j, (x2, y2, w2, h2) in enumerate(components):
            if i == j:
                continue

            if (
                x >= x2
                and y >= y2
                and x + w <= x2 + w2
                and y + h <= y2 + h2
            ):
                inside_other = True
                break

        if inside_other:
            continue

        center_x = x + w // 2
        center_y = y + h // 2

        objects.append([center_x, center_y, w, h])

    return np.array(objects, dtype=np.int32).reshape(-1, 4)


def get_distance(objects, cam_width, cam_height, tolerance_size):
    """Return the largest bounding box's offset toward the image center.

    Args:
        objects: Array with shape (N, 4), containing
            (center_x, center_y, width, height) for each object.
        cam_width: Image width in pixels.
        cam_height: Image height in pixels.
        tolerance_size: Per-axis threshold in pixels.
            Return offsets if either absolute offset reaches this threshold.

    Returns:
        (horizontal, vertical) offsets in pixels.
        Positive values indicate an object left of or above the image center.
        Returns None if no objects exist or both offsets are below the threshold.
    """
    if objects.size > 0:
        biggest = objects[(objects[:,2] * objects[:,3]).argmax()]
        distance_horizontal = int(-(biggest[0] - cam_width//2))
        distance_perpendicular = int(-(biggest[1] - cam_height//2))
        if abs(distance_horizontal) >= tolerance_size or abs(distance_perpendicular) >= tolerance_size:
            return (distance_horizontal, distance_perpendicular)
    return None