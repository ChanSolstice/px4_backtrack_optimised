#!/usr/bin/env python3
"""Host tests for recorder lifetime; no ROS installation or master required."""

import fcntl
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[2] / "scripts/record_flight_log.py"
SPEC = importlib.util.spec_from_file_location("record_flight_log", SCRIPT)
recorder_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = recorder_module
SPEC.loader.exec_module(recorder_module)


def spawn_child_process(command, **kwargs):
    del command
    return subprocess.Popen(
        [sys.executable, "-c", "import signal,time; signal.signal(signal.SIGINT, lambda *_: exit(0)); time.sleep(60)"],
        **kwargs)


class ConfigTest(unittest.TestCase):
    def test_default_limits_and_absolute_path(self):
        config = recorder_module.Config.from_mapping({})
        self.assertTrue(config.output_dir.is_absolute())
        self.assertEqual(config.split_size_mb * config.max_splits, 1024)
        self.assertEqual(config.buffer_mb, 32)
        self.assertEqual(config.min_space_mb, 256)
        self.assertEqual(config.keep_runs, 5)
        self.assertEqual(recorder_module.Config.from_mapping({"log_output_dir": "/mnt/flight logs"}).output_dir,
                         Path("/mnt/flight logs"))

    def test_bad_configuration_is_rejected(self):
        for params in ({"log_output_dir": "logs"}, {"log_output_dir": "~/logs"},
                       {"log_output_dir": 3}, {"log_enable": "false"},
                       {"log_topics": " /odom"}, {"log_topics": []},
                       {"log_topics": ["relative"]}, {"log_topics": ["/has space"]},
                       {"log_topics": ["/"]}, {"log_topics": ["/bad\x00name"]}):
            with self.subTest(params=params), self.assertRaises(ValueError):
                recorder_module.Config.from_mapping(params)
        for key in ("log_split_size_mb", "log_max_splits", "log_buffer_mb",
                    "log_min_space_mb", "log_keep_runs"):
            for value in (0, -1, 1.5, True, "16"):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    recorder_module.Config.from_mapping({key: value})

    def test_one_configuration_source_and_extra_topics(self):
        params = {"log_enable": False, "log_output_dir": "/mnt/logs", "log_topics": ["/slam/status"]}
        values = {"~config_namespace": "/flight/backtrack", "/flight/backtrack": params}
        config, namespace, effective = recorder_module.Config.load(lambda key, default: values.get(key, default))
        self.assertFalse(config.enabled)
        self.assertEqual(namespace, "/flight/backtrack")
        self.assertEqual(effective, params)
        self.assertIn("/slam/status", config.topics)
        self.assertTrue(set(recorder_module.DEFAULT_TOPICS).issubset(config.topics))

    def test_bag_command_keeps_paths_with_spaces_as_one_argument(self):
        config = recorder_module.Config.from_mapping({"log_split_size_mb": 12, "log_max_splits": 3})
        command = recorder_module.bag_command(config, Path("/mnt/logs/run with spaces"))
        self.assertEqual(command[command.index("-O") + 1], "/mnt/logs/run with spaces/flight")
        self.assertEqual(command[command.index("--size") + 1], "12")
        self.assertEqual(command[command.index("--max-splits") + 1], "3")
        self.assertEqual(command[command.index("--min-space") + 1], "256M")
        for flag in ("--split", "--lz4", "--tcpnodelay", "--repeat-latched"):
            self.assertIn(flag, command)


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.recorders = []

    def tearDown(self):
        for recorder in self.recorders:
            recorder.close()
        self.temporary.cleanup()

    def make_recorder(self, **params):
        config = recorder_module.Config.from_mapping({"log_output_dir": str(self.root), **params})
        recorder = recorder_module.Recorder(config)
        self.recorders.append(recorder)
        return recorder

    def test_disabled_has_no_files_or_child(self):
        recorder = self.make_recorder(log_enable=False)
        popen = mock.Mock()
        recorder.start(popen=popen)
        popen.assert_not_called()
        self.assertEqual(list(self.root.iterdir()), [])
        self.assertEqual(recorder.status()["state"], "disabled")

    def test_unique_run_and_manifest_capture(self):
        config_path = self.root / "trajectory.yaml"
        config_path.write_text("v_max: 0.6\n", encoding="utf-8")
        first = self.make_recorder()
        first.params = {"trajectory_config_path": str(config_path), "rejected_value": float("nan")}
        second = self.make_recorder()
        for recorder in (first, second):
            recorder.start(popen=spawn_child_process)
            self.assertEqual(recorder.state, "recording", recorder.error)
        self.assertNotEqual(first.run_dir, second.run_dir)
        first.attach_configuration({"build_revision": "0123456", "node_instance_id": "first"})
        first.attach_configuration({"build_revision": "abcdef0", "node_instance_id": "second"})
        first.attach_configuration({"build_revision": "0123456", "node_instance_id": "first"})
        first.close()
        manifest = json.loads((first.run_dir / "manifest.json").read_text(encoding="utf-8"))
        self.assertTrue(manifest["closed"])
        self.assertEqual(manifest["trajectory_config"]["yaml"], "v_max: 0.6\n")
        self.assertEqual(manifest["effective_parameters"]["rejected_value"], "nan")
        self.assertEqual(manifest["runtime_configuration"]["build_revision"], "0123456")
        self.assertEqual(len(manifest["runtime_configurations"]), 2)
        self.assertEqual(manifest["command"][0:2], ["rosbag", "record"])
        self.assertIn("end_utc", manifest)
        self.assertIsNotNone(first.process.returncode)
        self.assertTrue((first.run_dir / "recorder.log").exists())

    def test_space_failure_does_not_start_child(self):
        recorder = self.make_recorder()
        popen = mock.Mock()
        with mock.patch.object(recorder_module.shutil, "disk_usage", return_value=mock.Mock(free=1)):
            recorder.start(popen=popen)
        popen.assert_not_called()
        self.assertEqual(recorder.state, "failed")
        self.assertIn("free space", recorder.error)
        self.assertEqual(list(self.root.iterdir()), [])

    def test_unwritable_target_and_missing_rosbag_are_contained(self):
        file_target = self.root / "is_a_file"
        file_target.touch()
        recorder = recorder_module.Recorder(recorder_module.Config.from_mapping({"log_output_dir": str(file_target)}))
        self.recorders.append(recorder)
        recorder.start()
        self.assertEqual(recorder.state, "failed")
        self.assertIsNone(recorder.process)
        recorder = self.make_recorder()
        recorder.start(popen=mock.Mock(side_effect=PermissionError("cannot execute rosbag")))
        self.assertEqual(recorder.state, "failed")
        manifest = json.loads((recorder.run_dir / "manifest.json").read_text(encoding="utf-8"))
        self.assertTrue(manifest["closed"])
        self.assertTrue(manifest["interrupted"])
        self.assertEqual(manifest["stop_reason"], "startup_failed")
        self.assertIsNone(recorder.lock)

    def test_low_space_heartbeat_reports_suspension(self):
        recorder = self.make_recorder()
        recorder.start(popen=spawn_child_process)
        with mock.patch.object(recorder_module.shutil, "disk_usage", return_value=mock.Mock(free=1)):
            self.assertEqual(recorder.status()["state"], "insufficient_space")

    def test_unexpected_child_exit_is_recorded(self):
        recorder = self.make_recorder()
        recorder.start(popen=lambda command, **kwargs: subprocess.Popen([sys.executable, "-c", "exit(7)"], **kwargs))
        recorder.process.wait(timeout=2)
        recorder.poll()
        self.assertEqual(recorder.state, "failed")
        manifest = json.loads((recorder.run_dir / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["child_result"]["exit_code"], 7)
        self.assertEqual(manifest["stop_reason"], "recorder_exited")
        self.assertTrue(manifest["interrupted"])

    def test_retention_never_removes_active_unknown_or_unclosed_runs(self):
        def session(name, closed=True, owner=recorder_module.OWNER):
            path = self.root / name
            path.mkdir()
            (path / ".active.lock").touch()
            recorder_module.write_manifest(path / "manifest.json",
                                           {"owner": owner, "schema_version": 1, "closed": closed})
            return path
        oldest = session("run_000_closed")
        active = session("run_001_active")  # Even a stale 'closed' marker cannot defeat flock.
        unclosed = session("run_002_unclosed", closed=False)
        unknown = session("run_003_other", owner="unrelated")
        newest = session("run_004_closed")
        with (active / ".active.lock").open("r+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            removed = recorder_module.prune_closed_runs(self.root, keep_runs=3)
        self.assertEqual(removed, [str(oldest)])
        self.assertFalse(oldest.exists())
        for path in (active, unclosed, unknown, newest):
            self.assertTrue(path.exists())

    def test_retention_does_not_follow_symlinks(self):
        other = self.root / "external"
        other.mkdir()
        (self.root / "run_symlink").symlink_to(other, target_is_directory=True)
        self.assertEqual(recorder_module.prune_closed_runs(self.root, keep_runs=1), [])
        self.assertTrue(other.exists())


class ProcessStopTest(unittest.TestCase):
    def test_sigint_gracefully_reaps_real_child(self):
        code = "import signal,time; signal.signal(signal.SIGINT,lambda *_: exit(0)); print('ready',flush=True); time.sleep(60)"
        process = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE,
                                   start_new_session=True, text=True)
        try:
            self.assertEqual(process.stdout.readline().strip(), "ready")
            result = recorder_module.stop_process(process, timeout_s=1.0)
            self.assertEqual(result["exit_code"], 0)
            self.assertEqual(result["escalations"], [])
        finally:
            if process.poll() is None:
                recorder_module.stop_process(process, timeout_s=0.01)
            process.stdout.close()

    def test_sigint_ignoring_child_gets_bounded_termination(self):
        code = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); print('ready',flush=True); time.sleep(60)"
        process = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE,
                                   start_new_session=True, text=True)
        try:
            self.assertEqual(process.stdout.readline().strip(), "ready")
            started = time.monotonic()
            result = recorder_module.stop_process(process, timeout_s=0.1)
            self.assertIsNotNone(process.returncode)
            self.assertEqual(result["escalations"], ["SIGTERM"])
            self.assertLess(time.monotonic() - started, 3.0)
        finally:
            if process.poll() is None:
                recorder_module.stop_process(process, timeout_s=0.01)
            process.stdout.close()

    def test_unresponsive_child_gets_sigkill_and_is_reaped(self):
        code = ("import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); "
                "signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready',flush=True); time.sleep(60)")
        process = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE,
                                   start_new_session=True, text=True)
        try:
            self.assertEqual(process.stdout.readline().strip(), "ready")
            started = time.monotonic()
            result = recorder_module.stop_process(process, timeout_s=0.1)
            self.assertIsNotNone(process.returncode)
            self.assertEqual(result["escalations"], ["SIGTERM", "SIGKILL"])
            self.assertNotIn("error", result)
            self.assertLess(time.monotonic() - started, 4.0)
        finally:
            if process.poll() is None:
                recorder_module.stop_process(process, timeout_s=0.01)
            process.stdout.close()


if __name__ == "__main__":
    unittest.main()
