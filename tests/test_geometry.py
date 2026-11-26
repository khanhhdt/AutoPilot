import unittest

import cv2
import numpy as np

from src.control.pure_pursuit import PurePursuit
from src.perception.lane_detection import LaneState, _ipm_matrices, fit_lanes


class LaneGeometryTests(unittest.TestCase):
    def test_ipm_maps_trapezoid_to_rectangle_and_back(self):
        width, height = 400, 120
        trapezoid = np.float32([[32, 119], [368, 119], [232, 0], [168, 0]])
        rectangle = np.float32([[0, 119], [399, 119], [399, 0], [0, 0]])
        forward, inverse = _ipm_matrices(width, height)
        warped = cv2.perspectiveTransform(trapezoid[None, :, :], forward)
        np.testing.assert_allclose(warped[0], rectangle, atol=1e-3)
        np.testing.assert_allclose(cv2.perspectiveTransform(warped, inverse)[0], trapezoid, atol=1e-3)

    def test_polyfit_recovers_two_slanted_boundaries(self):
        mask = np.zeros((160, 320), np.uint8)
        y0 = 80
        for y in range(y0, len(mask)):
            left = round(96 - .2 * (y - y0))
            right = round(224 + .2 * (y - y0))
            mask[y, left-2:left+3] = 255
            mask[y, right-2:right+3] = 255
        left_fit, right_fit = fit_lanes(mask, roi_top=.5)
        np.testing.assert_allclose(np.polyval(left_fit, [12, 65]), [93.6, 83.], atol=1.)
        np.testing.assert_allclose(np.polyval(right_fit, [12, 65]), [226.4, 237.], atol=1.)

    def test_empty_mask_has_no_fit(self):
        self.assertEqual(fit_lanes(np.zeros((120, 240), np.uint8)), (None, None))

    def test_pursuit_turns_toward_lane_center_and_limits_first_step(self):
        for offset in (-30, 0, 30):
            with self.subTest(offset=offset):
                lane = LaneState(
                    detected=True, left_fit=np.array([0., 160 + offset]), right_fit=np.array([0., 340 + offset]),
                    offset_px=offset, center_px=250 + offset, mask=np.zeros((300, 500), np.uint8),
                    image_width=500, image_height=300, roi_top=.5,
                    overlay=np.zeros((300, 500, 3), np.uint8),
                )
                result = PurePursuit().step(lane, speed_kmh=10., dt=.05)
                self.assertIsNotNone(result)
                self.assertEqual(np.sign(result.steer), np.sign(offset))
                self.assertLessEqual(abs(result.steer), .025)
                self.assertAlmostEqual(result.lookahead_m, 8.)
