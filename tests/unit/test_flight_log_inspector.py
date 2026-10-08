"""Offline report tests: no ROS installation or generated message package required."""

import importlib.util
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace as Message
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "inspect_flight_log.py"
SPEC = importlib.util.spec_from_file_location("inspect_flight_log", SCRIPT)
log = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(log)


def control(**changes):
    values = dict(
        header=Message(seq=1), node_instance_id="node-A", tick_sequence=1,
        flight_id=2, plan_id=3, steady_time=10.0, tick_interval=.05, tick_cost=.002,
        current_state="ReturnActive", fcu_mode="OFFBOARD", armed=True,
        inputs_fresh=True, odom_valid=True, localization_reset=False, tick_clock_reset=False,
        odom_receipt_age=.01, state_receipt_age=.3,
        actual_velocity=Message(x=.3, y=.4, z=0.), projection_valid=True,
        projected_s=1.0, target_s=1.2, cross_track_error=.08,
        trajectory_valid=True, reference_time=3.0, clock_rate=.8,
        fallback_valid=False, fallback_index=0, fallback_count=0,
        route_projection_valid=True, route_distance=.12,
        route_prediction_valid=True, route_predicted_deviation=.2,
        route_safety_hold=False, input_hold=False, yaw_alignment_timeout=False,
        auto_land_yield=False, last_trigger="low_battery", last_trigger_reason="threshold held",
        input_failure_reason="", odometry_rejection_reason="", rc_loss=False, low_battery=True,
        operation="trajectory ready", operation_sequence=1,
        operation_steady_time=9.9, recorded_points=120,
        planning_requested=False, planning_inflight=False,
        mode_request_present=False, mode_request="")
    values.update(changes)
    return Message(**values)


def planning(**changes):
    values = dict(
        header=Message(seq=1), node_instance_id="node-A", flight_id=2, plan_id=3,
        steady_time_s=10., phase="started", status="", message="", success=False,
        fallback_ready=False, generation_cost_ms=0., duration_s=0., path_length_m=0.,
        max_speed_m_s=0., max_yaw_rate_rad_s=0., motion_speed_scale=1.,
        motion_limits_verified=False, corridor_verified=False,
        raw_position=[Message(x=0., y=1., z=2.), Message(x=1., y=1., z=2.)],
        raw_yaw_rad=[.1, .2], raw_timestamp_s=[3., 4.],
        effective_config="v_max: 0.6\na_max: 0.3\n", acceleration_budget_scale=.5)
    values.update(changes)
    return Message(**values)


class FlightLogInspectorTest(unittest.TestCase):
    def test_numeric_chunk_order_preserves_paused_and_reset_ros_time(self):
        paused = [(10., Path('/run/flight_10.bag'), 1), (10., Path('/run/flight_9.bag'), 1),
                  (10., Path('/run/flight_7.bag'), 1), (10., Path('/run/flight_13.bag'), 1)]
        ordered = log.order_bag_chunks(paused)
        self.assertEqual([p.name for _, p, _ in ordered],
                         ['flight_7.bag', 'flight_9.bag', 'flight_10.bag', 'flight_13.bag'])
        reset = [(5., Path('/run/flight_1.bag'), 1), (20., Path('/run/flight_0.bag'), 1),
                 (1., Path('/earlier/single.bag'), 1)]
        self.assertEqual([str(p) for _, p, _ in log.order_bag_chunks(reset)],
                         ['/earlier/single.bag', '/run/flight_0.bag', '/run/flight_1.bag'])

    def test_speed_route_progress_and_timing_are_grouped(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.PLANNING_TOPIC, planning(), 99.9)
        inspector.consume(log.CONTROL_TOPIC, control(), 100.)
        inspector.consume(log.CONTROL_TOPIC, control(tick_sequence=2, steady_time=10.05), 100.05)
        report = inspector.report()
        flight = next(iter(report["flights"].values()))
        metrics = flight["metrics"]
        self.assertAlmostEqual(metrics["actual_speed_m_s"]["max"], .5)
        self.assertEqual(metrics["route_distance_m"]["max"], .12)
        self.assertEqual(metrics["cross_track_error_m"]["max"], .08)
        self.assertAlmostEqual(metrics["reference_progress_lead_m"]["max"], .2)
        self.assertAlmostEqual(metrics["observed_diagnostic_interval_s"]["max"], .05)
        self.assertEqual(next(iter(report["plans"].values()))["control_samples"], 2)
        self.assertTrue(any("received log samples" in item for item in report["interpretation"]))

    def test_sequence_gap_reset_and_operation_deduplication(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.CONTROL_TOPIC, control(), 100.)
        inspector.consume(log.CONTROL_TOPIC, control(tick_sequence=4, steady_time=10.15), 100.15)
        inspector.consume(log.CONTROL_TOPIC, control(tick_sequence=5, steady_time=10.2,
                          input_hold=True, inputs_fresh=False, operation_sequence=2,
                          operation="invalid input hold"), 100.2)
        inspector.consume(log.CONTROL_TOPIC, control(tick_sequence=1, steady_time=11.), 101.)
        report = inspector.report()
        self.assertEqual(report["control_sequence"]["missing_between_samples"], 2)
        self.assertEqual(report["control_sequence"]["resets"], 1)
        operations = [event for event in inspector.events if event["category"] == "operation"]
        self.assertEqual(len(operations), 3)
        self.assertTrue(any(event["category"] == "input_hold" and "True" in event["message"]
                            for event in inspector.events))

    def test_invalid_values_are_not_reported_as_normal_zero(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.CONTROL_TOPIC, control(actual_velocity=Message(x=float("nan"), y=0., z=0.),
                          cross_track_error=float("inf"), projected_s=float("nan")), 100.)
        inspector.consume(log.CONTROL_TOPIC, control(tick_sequence=2, projection_valid=False,
                          route_projection_valid=False, route_prediction_valid=False,
                          cross_track_error=0., route_distance=0., route_predicted_deviation=0.), 100.1)
        metrics = next(iter(inspector.report()["flights"].values()))["metrics"]
        self.assertEqual(metrics["actual_speed_m_s"]["invalid_samples"], 1)
        self.assertIsNone(metrics["cross_track_error_m"]["max"])
        self.assertEqual(metrics["cross_track_error_m"]["invalid_samples"], 1)
        self.assertEqual(metrics["route_distance_m"]["samples"], 1)
        self.assertEqual(metrics["route_distance_m"]["max"], .12)

    def test_missing_schema_field_is_explicit(self):
        message = control()
        del message.route_distance
        inspector = log.FlightLogInspector()
        inspector.consume(log.CONTROL_TOPIC, message, 100.)
        inspector.consume(log.CONTROL_TOPIC, message, 100.1)
        missing = [item for item in inspector.report()["issues"] if item["code"] == "missing_field"]
        self.assertEqual(len(missing), 1)
        self.assertIn("route_distance", missing[0]["detail"])

    def test_restarted_nodes_do_not_merge_flight_or_failed_planning_inputs(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.CONTROL_TOPIC, control(tick_sequence=9), 100.)
        inspector.consume(log.PLANNING_TOPIC, planning(phase="finished", status="FIT_FAILED"), 100.01)
        inspector.consume(log.CONTROL_TOPIC, control(node_instance_id="node-B", steady_time=.1), 110.)
        inspector.consume(log.PLANNING_TOPIC, planning(node_instance_id="node-B", phase="finished",
                          status="OK", success=True,
                          raw_position=[Message(x=2., y=1., z=2.), Message(x=3., y=1., z=2.)]), 110.01)
        report = inspector.report()
        self.assertEqual(len(report["flights"]), 2)
        self.assertEqual(len(report["plans"]), 2)
        self.assertEqual(report["control_sequence"]["missing_between_samples"], 0)
        self.assertEqual(report["control_sequence"]["resets"], 0)
        self.assertEqual(len(inspector.planning_inputs), 2)
        self.assertEqual(report["plans"]["node-A:2:3"]["result"]["status"], "FIT_FAILED")

    def test_conflicting_same_plan_inputs_are_preserved_and_reported(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.PLANNING_TOPIC, planning(), 99.)
        inspector.consume(log.PLANNING_TOPIC, planning(phase="finished",
                          raw_position=[Message(x=9., y=1., z=2.), Message(x=10., y=1., z=2.)]), 100.)
        report = inspector.report()
        self.assertTrue(any(item["code"] == "conflicting_planning_inputs" for item in report["issues"]))
        original = next(iter(inspector.planning_inputs.values()))["planning_trace"]["raw_position"]
        self.assertEqual(original[0]["x"], 0.)
        alternative = report["plans"]["node-A:2:3"]["conflicting_inputs"][0]
        self.assertEqual(alternative["planning_trace"]["raw_position"][0]["x"], 9.)

    def test_failed_planning_input_and_nonfinite_locations_are_preserved(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.PLANNING_TOPIC, planning(raw_yaw_rad=[float("nan"), .2]), 99.)
        inspector.consume(log.PLANNING_TOPIC, planning(phase="finished", status="FIT_FAILED",
                          message="原始输入无法通过走廊验证", fallback_ready=True), 100.)
        with tempfile.TemporaryDirectory() as directory:
            summary = log.write_reports(inspector, Path(directory))
            plan = next(iter(summary["plans"].values()))
            snapshot = json.loads((Path(directory) / plan["input_file"]).read_text())
            self.assertEqual(snapshot["input_source_phase"], "started")
            self.assertEqual(snapshot["planning_trace"]["raw_position"][1]["x"], 1.)
            self.assertIsNone(snapshot["planning_trace"]["raw_yaw_rad"][0])
            self.assertEqual(snapshot["nonfinite_fields"]["raw_yaw_rad[0]"], "NaN")
            self.assertEqual(plan["result"]["status"], "FIT_FAILED")
            self.assertIn("FIT_FAILED", (Path(directory) / "events.log").read_text())
            json.loads((Path(directory) / "summary.json").read_text(),
                       parse_constant=lambda value: self.fail("Illegal JSON value: " + value))

    def test_late_subscriber_can_recover_input_from_discarded_trace(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.PLANNING_TOPIC, planning(phase="discarded", status="OK", success=True,
                          message="Result cancelled before delivery"), 100.)
        report = inspector.report()
        self.assertTrue(next(iter(report["plans"].values()))["input_available"])
        self.assertFalse(any(item["code"] == "missing_planning_input" for item in report["issues"]))
        self.assertEqual(next(iter(inspector.planning_inputs.values()))["input_source_phase"], "discarded")

    def test_mode_service_info_and_ros_errors_are_events(self):
        inspector = log.FlightLogInspector()
        for level, text in ((2, "[SetModeClient] OFFBOARD request started"),
                            (2, "[SetModeClient] OFFBOARD request result: failed"),
                            (2, "ordinary INFO"), (4, "localization old"), (8, "planning error")):
            inspector.consume("/rosout", Message(level=level, msg=text, name="/backtrack_node"), 100.)
        self.assertEqual(len([event for event in inspector.events if event["category"] == "mode_service"]), 2)
        self.assertEqual(inspector.report()["ros_errors"], 1)
        self.assertEqual(inspector.report()["ros_warnings"], 1)
        self.assertEqual(len(inspector.events), 4)

    def test_position_only_setpoints_do_not_create_velocity_statistics(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.CONTROL_TOPIC, control(), 100.)
        inspector.consume(log.SETPOINT_TOPIC, Message(type_mask=2552, velocity=Message(x=0., y=0., z=0.)), 100.01)
        inspector.consume(log.SETPOINT_TOPIC, Message(type_mask=0, velocity=Message(x=.6, y=0., z=0.)), 100.02)
        flight = next(iter(inspector.report()["flights"].values()))
        self.assertEqual(flight["setpoint_samples"], 2)
        self.assertEqual(flight["metrics"]["published_velocity_feedforward_m_s"]["samples"], 1)
        self.assertEqual(flight["metrics"]["published_velocity_feedforward_m_s"]["max"], .6)

    def test_recorder_health_heartbeats_and_capacity_warnings(self):
        inspector = log.FlightLogInspector()
        inspector.consume(log.CONTROL_TOPIC, control(), 100.)
        status = dict(state="recording", run_id="run_a", output_dir="/logs/run_a",
                      error="", free_bytes=1000, pid=7, exit_code=None)
        for timestamp, free in ((100.1, 1000), (100.2, 900)):
            status["free_bytes"] = free
            inspector.consume(log.RECORDER_STATUS_TOPIC, Message(data=json.dumps(status)), timestamp)
        status.update(state="insufficient_space", free_bytes=100)
        inspector.consume(log.RECORDER_STATUS_TOPIC, Message(data=json.dumps(status)), 100.3)
        inspector.consume("/rosout", Message(level=4, msg="buffer exceeded; dropping old messages",
                          name="/record_12345"), 100.4)
        report = inspector.report()
        self.assertEqual(len(report["recorder_states"]), 2)
        self.assertEqual(report["recorder_runs"]["run_a"]["free_bytes"]["min"], 100)
        self.assertTrue(any(item["code"] == "recorder_unhealthy" for item in report["issues"]))
        self.assertIsNone(inspector.events[-1]["flight_id"])
        self.assertEqual(inspector.events[-1]["source"], "/record_12345")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "manifest.json").write_text("{}")
            (root / "recorder.log").write_text("normal line\n[WARN] buffer exceeded\n"
                                               "[ERROR] insufficient disk space\n")
            evidence = log.read_run_metadata(root, inspector)
            self.assertEqual(evidence["recorder_warning_line_count"], 2)
            self.assertEqual(evidence["recorder_warning_lines"][0]["line"], 2)
            self.assertTrue(any(item["code"] == "recorder_log_warnings" for item in inspector.issues))

    def test_missing_bag_chunks_and_active_file_are_reported_without_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("flight_2.bag", "flight_4.bag", "flight_5.bag.active"):
                (root / name).write_bytes(b"unchanged evidence")
            bags, run_directory, issues = log.discover_inputs([root])
            self.assertEqual(len(bags), 2)
            self.assertEqual(run_directory, root)
            codes = {item["code"] for item in issues}
            self.assertTrue({"missing_bag_prefix", "missing_bag_chunks", "unfinished_bag"} <= codes)
            for path in root.iterdir():
                self.assertEqual(path.read_bytes(), b"unchanged evidence")

    def test_cli_writes_report_for_absent_or_unfinished_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "flight_0.bag.active").write_bytes(b"unfinished")
            output = root / "report"
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(log.main([str(root), "--output-dir", str(output)]), 2)
            summary = json.loads((output / "summary.json").read_text())
            self.assertTrue(any(item["code"] == "unfinished_bag" for item in summary["issues"]))
            self.assertEqual((root / "flight_0.bag.active").read_bytes(), b"unfinished")

    def test_corrupted_bag_does_not_prevent_inspection_of_later_bags(self):
        class FakeBag:
            def __init__(self, path, mode):
                self.path = Path(path)

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def get_start_time(self):
                return 100. if self.path.name == "flight_0.bag" else 101.

            def read_messages(self):
                if self.path.name == "flight_0.bag":
                    yield log.CONTROL_TOPIC, control(), Message(to_sec=lambda: 100.)
                    raise OSError("truncated chunk")
                yield log.CONTROL_TOPIC, control(tick_sequence=2), Message(to_sec=lambda: 101.)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("flight_0.bag", "flight_1.bag"):
                (root / name).write_bytes(b"source evidence")
            with mock.patch.dict("sys.modules", {"rosbag": Message(Bag=FakeBag)}), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(log.main([str(root)]), 2)
            summary = json.loads((root / "summary.json").read_text())
            self.assertEqual(summary["flights"]["node-A:2"]["control_samples"], 2)
            self.assertFalse(summary["evidence"]["bags"][0]["read_ok"])
            self.assertTrue(summary["evidence"]["bags"][1]["read_ok"])
            self.assertTrue(any(item["code"] == "incomplete_bag_read" for item in summary["issues"]))
            for name in ("flight_0.bag", "flight_1.bag"):
                self.assertEqual((root / name).read_bytes(), b"source evidence")


if __name__ == "__main__":
    unittest.main()
