import unittest

from src.metrics import RunTelemetry


class TelemetryTests(unittest.TestCase):
    def test_uses_path_length_and_closing_rate(self):
        telemetry = RunTelemetry()
        key = (1, -1, "vehicle")
        for tick in range(5):
            row = telemetry.update(tick * .05, (tick * .1, 0., 0.), key, 10. - tick * .1)
        self.assertAlmostEqual(row["distance_m"], .4)
        self.assertAlmostEqual(row["ttc_s"], 4.8)

    def test_breaks_history_when_the_lane_or_stream_changes(self):
        telemetry = RunTelemetry()
        key = (1, -1, "vehicle")
        for tick in range(5):
            telemetry.update(tick * .05, (0., 0., 0.), key, 10. - tick * .1)
        self.assertIsNone(telemetry.update(.25, (0., 0., 0.), (1, -2, "vehicle"), 9.5)["ttc_s"])
        self.assertIsNone(telemetry.update(.30, None)["ttc_s"])
        row = telemetry.update(.35, (10., 0., 0.), key, 9.)
        self.assertEqual(row["distance_m"], 0.)
        self.assertIsNone(row["ttc_s"])

    def test_opening_gap_has_no_ttc(self):
        telemetry = RunTelemetry()
        for tick in range(5):
            row = telemetry.update(tick * .05, (0., 0., 0.), (1, -1, "prop"), 7. + tick * .1)
        self.assertIsNone(row["ttc_s"])
