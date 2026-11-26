"""Semantic tags from CARLA 0.9.16. Palette images use OpenCV's BGR order."""

from dataclasses import dataclass

import numpy as np

ROAD_TAG = 1
ROAD_LINE_TAG = 24
VEHICLE_COLORS = {
    14: (0, 0, 142), 15: (0, 0, 70), 16: (0, 60, 100),
    17: (0, 80, 100), 18: (0, 0, 230), 19: (119, 11, 32),
}

PALETTE_RGB = {
    ROAD_TAG: (128, 64, 128), ROAD_LINE_TAG: (157, 234, 50),
    12: (220, 20, 60), 13: (255, 0, 0), 20: (110, 190, 160), 21: (170, 120, 50),
    **VEHICLE_COLORS,
}


@dataclass(frozen=True)
class SemanticMasks:
    road: np.ndarray
    road_line: np.ndarray
    vehicles: np.ndarray
    pedestrians: np.ndarray
    props: np.ndarray


def masks(tags: np.ndarray) -> SemanticMasks:
    return SemanticMasks(
        road=tags == ROAD_TAG,
        road_line=tags == ROAD_LINE_TAG,
        vehicles=np.isin(tags, tuple(VEHICLE_COLORS)),
        pedestrians=(tags == 12) | (tags == 13),
        props=(tags == 20) | (tags == 21),
    )


def colorize(tags: np.ndarray) -> np.ndarray:
    palette = np.zeros((256, 3), dtype=np.uint8)
    for tag, rgb in PALETTE_RGB.items():
        palette[tag] = rgb[::-1]
    return palette[tags]
