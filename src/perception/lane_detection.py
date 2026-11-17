from dataclasses import dataclass

import cv2
import numpy as np

ROAD_TAG = 1
ROAD_LINE_TAG = 24


@dataclass
class LaneState:
    detected: bool
    offset_px: float
    center_px: float
    mask: np.ndarray
    overlay: np.ndarray
    left_fit: np.ndarray | None
    right_fit: np.ndarray | None
    fresh: bool = False
    missed_frames: int = 0
    image_width: int = 0
    image_height: int = 0
    roi_top: float = 0.55


def lane_pair_valid(left: np.ndarray, right: np.ndarray, width: int, height: int) -> bool:
    """Reject crossed boundaries and implausible lane widths in the near ROI."""
    ys = np.linspace(0.4 * (height - 1), height - 1, 12)
    left_x = np.polyval(left, ys)
    right_x = np.polyval(right, ys)
    widths = right_x - left_x
    return bool(
        np.isfinite(left_x).all() and np.isfinite(right_x).all()
        and np.all(widths > 0)
        and 0.2 * width <= widths[-1] <= 0.8 * width
        and -0.25 * width <= left_x[-1] < right_x[-1] <= 1.25 * width
    )


def lane_mask(
    bgr: np.ndarray,
    roi_top: float = 0.55,
    road_mask: np.ndarray | None = None,
    edge_mode: str = "road",
    edge_dilate: int = 11,
) -> np.ndarray:
    height, width = bgr.shape[:2]
    y0 = int(height * roi_top)
    roi = bgr[y0:]
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)
    edges = cv2.Canny(cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY), 50, 150)
    hls = cv2.cvtColor(blurred, cv2.COLOR_BGR2HLS)
    lightness = hls[:, :, 1]
    background = lightness[road_mask[y0:] > 0] if road_mask is not None else lightness.ravel()
    white_threshold = max(195., min(245., float(np.percentile(background, 80)) + 15.)) if background.size else 195.
    white = ((lightness > white_threshold) & (hls[:, :, 2] < 55)).astype(np.uint8) * 255
    yellow = (
        (hls[:, :, 0] >= 15)
        & (hls[:, :, 0] <= 40)
        & (hls[:, :, 1] > 90)
        & (hls[:, :, 2] > 80)
    ).astype(np.uint8) * 255
    mask = cv2.bitwise_or(white, yellow)
    if edge_mode == "road" and road_mask is not None:
        edge_source = road_mask[y0:]
    else:
        edge_source = cv2.dilate(mask, np.ones((edge_dilate, edge_dilate), np.uint8))
    edge_support = cv2.bitwise_and(
        cv2.dilate(edges, np.ones((3, 3), np.uint8)),
        cv2.dilate(edge_source, np.ones((3, 3), np.uint8)),
    )
    mask = cv2.bitwise_or(mask, edge_support)
    if road_mask is not None:
        mask = cv2.bitwise_and(mask, road_mask[y0:])
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    full = np.zeros((height, width), np.uint8)
    full[y0:] = mask
    return full


def _ipm_matrices(width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    source = np.float32([
        [width * 0.08, height - 1],
        [width * 0.92, height - 1],
        [width * 0.58, 0],
        [width * 0.42, 0],
    ])
    destination = np.float32([
        [0, height - 1],
        [width - 1, height - 1],
        [width - 1, 0],
        [0, 0],
    ])
    forward = cv2.getPerspectiveTransform(source, destination)
    return forward, np.linalg.inv(forward)


def _collect_side_points(
    roi: np.ndarray,
    base: int | None,
    window_height: int,
    margin: int,
    min_pixels: int,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    if base is None:
        return None, None
    height, width = roi.shape
    current = float(base)
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for i in range(height // window_height):
        y_low = height - (i + 1) * window_height
        y_high = height - i * window_height
        x_low = max(0, int(current) - margin)
        x_high = min(width, int(current) + margin)
        yy, xx = np.nonzero(roi[y_low:y_high, x_low:x_high])
        if xx.size < min_pixels:
            continue
        xs.append(xx + x_low)
        ys.append(yy + y_low)
        current = float(np.mean(xx + x_low))
    if not xs:
        return None, None
    return (
        np.concatenate(xs).astype(np.float32),
        np.concatenate(ys).astype(np.float32),
    )


def _fit_side(
    roi: np.ndarray,
    base: int | None,
    window_height: int,
    margin: int,
    min_pixels: int,
) -> np.ndarray | None:
    xs, ys = _collect_side_points(roi, base, window_height, margin, min_pixels)
    if xs is None or xs.size < 10:
        return None
    height, width = roi.shape
    fit = np.polyfit(ys, xs, 1)
    if abs(float(fit[0])) > 3.0:
        return None
    bottom_x = float(np.polyval(fit, height - 1))
    if not -0.25 * width <= bottom_x < 1.25 * width:
        return None
    return fit


def _pick_base(
    histogram: np.ndarray,
    low: int,
    high: int,
    center: int,
    min_pixels: int,
    max_width: int,
) -> int | None:
    columns = np.nonzero(histogram[low:high] >= min_pixels)[0] + low
    if columns.size == 0:
        return None
    groups = np.split(columns, np.nonzero(np.diff(columns) > 1)[0] + 1)
    best_distance = None
    best_x = None
    for group in groups:
        if group[-1] - group[0] + 1 > max_width:
            continue
        x = int(group[np.argmax(histogram[group])])
        distance = abs(x - center)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_x = x
    return best_x


def fit_lanes(
    mask: np.ndarray,
    roi_top: float = 0.55,
    n_windows: int = 6,
    margin: int = 50,
    min_pixels: int = 15,
    min_base_pixels: int = 5,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    height, width = mask.shape
    y0 = int(height * roi_top)
    roi = mask[y0:] > 0
    roi_height = roi.shape[0]
    if roi_height < n_windows:
        return None, None
    window_height = max(1, roi_height // n_windows)
    center = width // 2
    left_low = max(0, center - int(width * 0.45))
    left_high = max(0, center - int(width * 0.10))
    right_low = min(width, center + int(width * 0.10))
    right_high = min(width, center + int(width * 0.45))
    max_width = max(3, int(width * 0.12))
    half_band = max(3, roi_height // 12)
    left_candidates: list[int] = []
    right_candidates: list[int] = []
    for fraction in (0.9, 0.78, 0.66, 0.54):
        y_center = int(roi_height * fraction)
        y_low = max(0, y_center - half_band)
        y_high = min(roi_height, y_center + half_band)
        histogram = np.count_nonzero(roi[y_low:y_high], axis=0)
        left_candidate = _pick_base(
            histogram, left_low, left_high, center, min_base_pixels, max_width
        )
        right_candidate = _pick_base(
            histogram, right_low, right_high, center, min_base_pixels, max_width
        )
        if left_candidate is not None:
            left_candidates.append(left_candidate)
        if right_candidate is not None:
            right_candidates.append(right_candidate)
    left_base = min(left_candidates, key=lambda x: abs(x - center)) if left_candidates else None
    right_base = min(right_candidates, key=lambda x: abs(x - center)) if right_candidates else None
    left_fit = _fit_side(roi, left_base, window_height, margin, min_pixels)
    right_fit = _fit_side(roi, right_base, window_height, margin, min_pixels)
    return left_fit, right_fit


def _pick_base_robust(
    histogram: np.ndarray,
    low: int,
    high: int,
    min_pixels: int,
    max_width: int,
) -> int | None:
    columns = np.nonzero(histogram[low:high] >= min_pixels)[0] + low
    if columns.size == 0:
        return None
    groups = np.split(columns, np.nonzero(np.diff(columns) > 1)[0] + 1)
    best_score = -1
    best_x = None
    for group in groups:
        if group[-1] - group[0] + 1 > max_width:
            continue
        if group[0] <= low + 1 or group[-1] >= high - 2:
            continue
        score = int(histogram[group].sum())
        if score > best_score:
            best_score = score
            best_x = int(group[np.argmax(histogram[group])])
    return best_x


def _fit_camera_from_points(
    xs: np.ndarray | None,
    ys: np.ndarray | None,
    inverse: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray | None:
    if xs is None or ys is None or xs.size < 10:
        return None
    points = np.stack([xs, ys], axis=1).astype(np.float32).reshape(1, -1, 2)
    projected = cv2.perspectiveTransform(points, inverse)[0]
    valid = (
        np.isfinite(projected).all(axis=1)
        & (projected[:, 0] >= 0)
        & (projected[:, 0] < width)
        & (projected[:, 1] >= 0)
        & (projected[:, 1] < height)
    )
    if valid.sum() < 10:
        return None
    fit = np.polyfit(projected[valid, 1], projected[valid, 0], 1)
    if abs(float(fit[0])) > 3.0:
        return None
    bottom_x = float(np.polyval(fit, height - 1))
    if not -0.25 * width <= bottom_x < 1.25 * width:
        return None
    return fit


def fit_lanes_camera(
    mask: np.ndarray,
    inverse: np.ndarray,
    n_windows: int = 6,
    margin: int = 50,
    min_pixels: int = 15,
    min_base_pixels: int = 5,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    roi = mask > 0
    height, width = roi.shape
    if height < n_windows:
        return None, None
    window_height = max(1, height // n_windows)
    center = width // 2
    left_low = max(0, center - int(width * 0.45))
    left_high = max(0, center - int(width * 0.10))
    right_low = min(width, center + int(width * 0.10))
    right_high = min(width, center + int(width * 0.45))
    max_width = max(3, int(width * 0.12))
    half_band = max(3, height // 12)
    left_base = None
    right_base = None
    for fraction in (0.9, 0.78, 0.66, 0.54):
        y_center = int(height * fraction)
        y_low = max(0, y_center - half_band)
        y_high = min(height, y_center + half_band)
        histogram = np.count_nonzero(roi[y_low:y_high], axis=0)
        if left_base is None:
            left_base = _pick_base_robust(
                histogram, left_low, left_high, min_base_pixels, max_width
            )
        if right_base is None:
            right_base = _pick_base_robust(
                histogram, right_low, right_high, min_base_pixels, max_width
            )
        if left_base is not None and right_base is not None:
            break
    left_x, left_y = _collect_side_points(
        roi, left_base, window_height, margin, min_pixels
    )
    right_x, right_y = _collect_side_points(
        roi, right_base, window_height, margin, min_pixels
    )
    left_fit = _fit_camera_from_points(left_x, left_y, inverse, width, height)
    right_fit = _fit_camera_from_points(right_x, right_y, inverse, width, height)
    return left_fit, right_fit


class LaneDetector:
    def __init__(
        self,
        roi_top: float = 0.55,
        n_windows: int = 6,
        margin: int | None = None,
        min_pixels: int = 15,
        smooth: float = 0.85,
        max_missed: int = 5,
        hybrid: bool = True,
        edge_dilate: int = 11,
    ) -> None:
        self._roi_top = roi_top
        self._n_windows = n_windows
        self._margin = margin
        self._min_pixels = min_pixels
        self._smooth = smooth
        self._max_missed = max_missed
        self._hybrid = hybrid
        self._edge_dilate = edge_dilate
        self.reset()

    def reset(self) -> None:
        """Discard temporal fits after a sensor timeout or a skipped world tick."""
        self._left_fit: np.ndarray | None = None
        self._right_fit: np.ndarray | None = None
        self._left_missed = 0
        self._right_missed = 0

    def detect(self, bgr: np.ndarray, road_mask: np.ndarray | None = None) -> LaneState:
        height, width = bgr.shape[:2]
        y0 = int(height * self._roi_top)
        roi_height = height - y0
        forward, inverse = _ipm_matrices(width, roi_height)
        margin = (
            self._margin if self._margin is not None
            else max(1, round(width * 50 / 512))
        )
        modes = (
            ("road", self._edge_dilate),
            ("color", self._edge_dilate),
        ) if self._hybrid else (("road", self._edge_dilate),)
        candidates: list[tuple[np.ndarray, np.ndarray, np.ndarray, float, float]] = []
        fallback_mask = None
        for edge_mode, edge_dilate in modes:
            mask = lane_mask(bgr, self._roi_top, road_mask, edge_mode, edge_dilate)
            if fallback_mask is None:
                fallback_mask = mask
            bird_mask = cv2.warpPerspective(
                mask[y0:], forward, (width, roi_height), flags=cv2.INTER_NEAREST
            )
            left_fit, right_fit = fit_lanes_camera(
                bird_mask, inverse, self._n_windows, margin, self._min_pixels
            )
            if (left_fit is not None and right_fit is not None
                    and lane_pair_valid(left_fit, right_fit, width, roi_height)):
                left_x = float(np.polyval(left_fit, roi_height - 1))
                right_x = float(np.polyval(right_fit, roi_height - 1))
                candidates.append((left_fit, right_fit, mask, left_x, right_x))
        if candidates:
            if len(candidates) == 1:
                left_fit, right_fit, mask, _, _ = candidates[0]
            else:
                prev_width = None
                prev_center = None
                if self._left_fit is not None and self._right_fit is not None:
                    prev_left = float(np.polyval(self._left_fit, roi_height - 1))
                    prev_right = float(np.polyval(self._right_fit, roi_height - 1))
                    prev_width = prev_right - prev_left
                    prev_center = 0.5 * (prev_left + prev_right)
                expected_width = (
                    prev_width
                    if prev_width is not None and 0.2 * width <= prev_width <= 0.8 * width
                    else 0.4 * width
                )
                expected_center = prev_center if prev_center is not None else width / 2.0

                def score(candidate):
                    _, _, _, left_x, right_x = candidate
                    width_px = right_x - left_x
                    center = 0.5 * (left_x + right_x)
                    return (
                        abs(width_px - expected_width) / width
                        + abs(center - expected_center) / width
                    )

                left_fit, right_fit, mask, _, _ = min(candidates, key=score)
        else:
            left_fit = None
            right_fit = None
            mask = fallback_mask if fallback_mask is not None else lane_mask(
                bgr, self._roi_top, road_mask
            )

        fresh = left_fit is not None and right_fit is not None
        if left_fit is not None and self._left_fit is not None:
            left_fit = self._smooth * self._left_fit + (1.0 - self._smooth) * left_fit
        if right_fit is not None and self._right_fit is not None:
            right_fit = self._smooth * self._right_fit + (1.0 - self._smooth) * right_fit

        if left_fit is not None:
            self._left_missed = 0
        else:
            self._left_missed += 1
            left_fit = self._left_fit if self._left_missed <= self._max_missed else None
        if right_fit is not None:
            self._right_missed = 0
        else:
            self._right_missed += 1
            right_fit = self._right_fit if self._right_missed <= self._max_missed else None
        center_x = None
        if (left_fit is not None and right_fit is not None
                and lane_pair_valid(left_fit, right_fit, width, roi_height)):
            left_x = float(np.polyval(left_fit, roi_height - 1))
            right_x = float(np.polyval(right_fit, roi_height - 1))
            center_x = 0.5 * (left_x + right_x)
        else:
            left_fit = right_fit = None
            fresh = False
        self._left_fit = left_fit
        self._right_fit = right_fit
        detected = center_x is not None
        offset = float(center_x - width / 2.0) if detected else 0.0
        overlay = self._draw(bgr, mask, left_fit, right_fit, y0, roi_height, offset, detected)
        return LaneState(
            detected=detected,
            offset_px=offset,
            center_px=float(center_x) if detected else width / 2.0,
            mask=mask,
            overlay=overlay,
            left_fit=left_fit,
            right_fit=right_fit,
            fresh=fresh,
            missed_frames=max(self._left_missed, self._right_missed),
            image_width=width,
            image_height=height,
            roi_top=self._roi_top,
        )

    def _draw(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        left_fit: np.ndarray | None,
        right_fit: np.ndarray | None,
        y0: int,
        roi_height: int,
        offset: float,
        detected: bool,
    ) -> np.ndarray:
        overlay = bgr.copy()
        overlay[mask > 0] = (0, 255, 0)
        for fit in (left_fit, right_fit):
            if fit is None:
                continue
            ys = np.linspace(0, roi_height - 1, 30)
            xs = np.polyval(fit, ys)
            points = np.stack([xs, ys + y0], axis=1).astype(np.int32)
            cv2.polylines(overlay, [points], False, (0, 0, 255), 2)
        if detected:
            center = int(round(bgr.shape[1] / 2.0 + offset))
            cv2.circle(overlay, (center, bgr.shape[0] - 4), 5, (255, 0, 0), -1)
        text = f"offset {offset:+.1f}px" if detected else "lane: lost"
        cv2.putText(
            overlay, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
        )
        return overlay
