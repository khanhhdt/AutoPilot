"""Independent RoadLines component fits for auditing the sliding-window GT.

These candidates are diagnostic only: fitting a short or occluded marking can
still give an unreliable extrapolation. They never replace benchmark labels.
"""

import cv2
import numpy as np


def component_lane_fits(mask: np.ndarray, roi_top: float = 0.55):
    height, width = mask.shape
    y0 = int(height * roi_top)
    roi = (mask[y0:] > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(roi, connectivity=8)
    candidates = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_HEIGHT] < max(6, round(height * 0.05)):
            continue
        ys, xs = np.nonzero(labels == label)
        if xs.size < 8:
            continue
        fit = np.polyfit(ys, xs, 1)
        bottom = float(np.polyval(fit, roi.shape[0] - 1))
        if abs(fit[0]) <= 3 and -0.25 * width <= bottom <= 1.25 * width:
            candidates.append((fit, bottom))
    left = [item for item in candidates if item[1] < width / 2]
    right = [item for item in candidates if item[1] > width / 2]
    return (
        max(left, key=lambda item: item[1])[0] if left else None,
        min(right, key=lambda item: item[1])[0] if right else None,
    )
