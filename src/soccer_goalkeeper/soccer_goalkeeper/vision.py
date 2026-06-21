import cv2

import numpy as np


def find_soccer_ball_contour(image, minimum_area):
    """Return the most ball-like white contour in a camera image."""
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # The ball body is bright and nearly colorless. Field lines and goalposts
    # share that color, so geometry filters below reject long, thin contours.
    mask = cv2.inRange(
        hsv_image,
        np.array([0, 0, 150], dtype=np.uint8),
        np.array([180, 85, 255], dtype=np.uint8)
    )
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < minimum_area or area > 350.0:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        if width == 0 or height == 0:
            continue

        aspect_ratio = width / height
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if not 0.55 <= aspect_ratio <= 1.8 or circularity < 0.35:
            continue

        candidates.append((circularity, area, contour))

    if not candidates:
        return None

    # Prefer the roundest candidate, then the larger candidate when shapes
    # have similar circularity.
    best_candidate = max(
        candidates,
        key=lambda candidate: (candidate[0], candidate[1])
    )
    return best_candidate[2]
