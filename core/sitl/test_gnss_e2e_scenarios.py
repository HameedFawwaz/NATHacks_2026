"""Fast reporting regressions; run with the same pymavlink Python as SITL."""
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import run_gnss_e2e_scenarios as e2e


class ReportingTests(unittest.TestCase):
    def test_control_false_alarm_fails_and_keeps_results(self):
        result = {"label": "control (unspoofed)", "behaviors_entered": ["GNSS_FAILOVER"],
                  "lowest_nav_trust": 0.1}
        with tempfile.TemporaryDirectory() as directory:
            argv = ["e2e", "--ardupilot", directory, "--log-dir", directory, "control"]
            with patch("sys.argv", argv), patch.object(e2e, "Sitl") as sitl, \
                    patch.dict(e2e.SCENARIOS, {"control": lambda *_: result}, clear=True):
                self.assertEqual(e2e.main(), 1)
                sitl.return_value.close.assert_called_once()
            self.assertEqual(json.loads((Path(directory) / "results.json").read_text()), [result])

    def test_dashboard_reads_live_telemetry_not_socket_status(self):
        expected = {"Connected": True, "Nav": {"Value": 0.2}}
        stream = io.BytesIO(b"data: " + json.dumps({"Snapshot": expected}).encode() + b"\n\n")
        with patch.object(e2e.urllib.request, "urlopen", return_value=stream) as request:
            self.assertEqual(e2e.dashboard_snapshot("http://localhost:8081"), expected)
            self.assertTrue(request.call_args.args[0].endswith("/api/telemetry"))

    def test_missing_dashboard_data_is_not_a_successful_snapshot(self):
        with patch.object(e2e.urllib.request, "urlopen", return_value=io.BytesIO(b"")):
            with self.assertRaises(e2e.ScenarioFailed):
                e2e.dashboard_snapshot("http://localhost:8081")

    def test_dead_detector_cannot_pass_as_healthy(self):
        class DeadDetector:
            status = {"true_pos": (0, 1), "behavior_log": [], "nav_trust": 1.0, "nav_at": 0}

            def pump(self, *_):
                pass

        with self.assertRaisesRegex(e2e.ScenarioFailed, "TRUST_NAV stopped"):
            e2e.run_window(DeadDetector(), 1, None, "control")


if __name__ == "__main__":
    unittest.main()
