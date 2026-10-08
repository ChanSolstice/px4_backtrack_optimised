#!/usr/bin/env python3
"""Supervise one ROS rosbag recorder; importing this file never imports ROS.

ROS parameters live under ~config_namespace (default /backtrack_node). This
process handles files and child lifetime; the flight control node does neither.
"""

import dataclasses
import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from typing import Any, Mapping, Optional


OWNER = "px4_backtrack.record_flight_log"
SCHEMA_VERSION = 1
# Noetic --repeat-latched can repeatedly reopen bags if their initial latched
# messages already exceed --size. Budget all built-in latched diagnostics/RViz
# for the supported <1 h recording, with serialization/connection headroom.
MIN_SPLIT_MB = 64
DEFAULT_TOPICS = (
    "/Odometry_px4", "/mavros/state", "/mavros/rc/in", "/mavros/battery",
    "/mavros/setpoint_raw/local", "/px4_backtrack/state", "/status_rc_loss",
    "/px4_backtrack/low_battery", "/px4_backtrack/debug/waypoints",
    "/px4_backtrack/debug/trajectory", "/px4_backtrack/log/control",
    "/px4_backtrack/log/planning", "/px4_backtrack/log/configuration",
    "/px4_backtrack/log/recorder_status", "/rosout", "/clock",
)


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def json_safe(value: Any) -> Any:
    """Keep even rejected nonfinite ROS parameters representable in strict JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    return str(value)


def write_manifest(path: Path, manifest: Mapping[str, Any]) -> None:
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(json_safe(manifest), ensure_ascii=False,
                                    indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


@dataclasses.dataclass(frozen=True)
class Config:
    enabled: bool = True
    output_dir: Path = dataclasses.field(
        default_factory=lambda: Path(os.environ.get("HOME", "/tmp")) / ".ros/backtrack_logs")
    split_size_mb: int = 64
    max_splits: int = 16
    buffer_mb: int = 32
    min_space_mb: int = 256
    keep_runs: int = 5
    topics: tuple = DEFAULT_TOPICS

    @classmethod
    def from_mapping(cls, params: Mapping[str, Any]) -> "Config":
        enabled = params.get("log_enable", True)
        if not isinstance(enabled, bool):
            raise ValueError("log_enable must be a boolean")
        directory = params.get("log_output_dir", "")
        if not isinstance(directory, str):
            raise ValueError("log_output_dir must be an absolute path string or empty")
        if directory and not Path(directory).is_absolute():
            raise ValueError("log_output_dir must be absolute; '~' is not expanded")
        defaults = cls()
        values = {}
        for parameter, field in (
            ("log_split_size_mb", "split_size_mb"), ("log_max_splits", "max_splits"),
            ("log_buffer_mb", "buffer_mb"), ("log_min_space_mb", "min_space_mb"),
            ("log_keep_runs", "keep_runs"),
        ):
            value = params.get(parameter, getattr(defaults, field))
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(parameter + " must be a positive integer")
            values[field] = value
        rate = params.get("record_max_hz", 15.0)
        if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or rate <= 0:
            raise ValueError("record_max_hz must be finite and positive for logging capacity validation")
        latched_budget = (math.ceil(3600 * rate) + 2) * 512 + 20000 * 256 + 4 * 1024 * 1024
        minimum_split_mb = max(MIN_SPLIT_MB, math.ceil(latched_budget / (1024 * 1024)))
        if values["split_size_mb"] < minimum_split_mb:
            raise ValueError("log_split_size_mb must be at least {} for the configured recording rate; "
                             "smaller splits can loop when rosbag repeats latched messages".format(minimum_split_mb))
        topics = params.get("log_topics", list(DEFAULT_TOPICS))
        if not isinstance(topics, (list, tuple)) or not topics:
            raise ValueError("log_topics must be a nonempty list")
        if any(not isinstance(topic, str) or
               not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", topic) for topic in topics):
            raise ValueError("log_topics entries must be absolute ROS topic names")
        # Mandatory diagnosis topics cannot disappear when extra sensor topics are added.
        topics = tuple(dict.fromkeys(DEFAULT_TOPICS + tuple(topics)))
        return cls(enabled=enabled, output_dir=Path(directory) if directory else defaults.output_dir,
                   topics=topics, **values)

    @classmethod
    def load(cls, get_param):
        namespace = get_param("~config_namespace", "/backtrack_node")
        if not isinstance(namespace, str) or not namespace.startswith("/"):
            raise ValueError("config_namespace must be an absolute ROS namespace")
        params = get_param(namespace, {})
        if not isinstance(params, dict):
            raise ValueError("configuration namespace must contain a parameter dictionary")
        return cls.from_mapping(params), namespace, params


def bag_command(config: Config, run_dir: Path) -> list:
    return ["rosbag", "record", "--split", "--size", str(config.split_size_mb),
            "--max-splits", str(config.max_splits), "--buffsize", str(config.buffer_mb),
            "--min-space", str(config.min_space_mb) + "M", "--repeat-latched", "--lz4",
            "--tcpnodelay", "-O", str(run_dir / "flight")] + list(config.topics)


def prune_closed_runs(root: Path, keep_runs: int) -> list:
    """Only our closed sessions are candidates; flock protects live supervisors."""
    sessions = []
    for directory in root.iterdir():
        if directory.is_symlink() or not directory.is_dir() or not directory.name.startswith("run_"):
            continue
        try:
            manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
            if manifest.get("owner") != OWNER or manifest.get("schema_version") != SCHEMA_VERSION:
                continue
            sessions.append((directory, manifest))
        except (OSError, ValueError, AttributeError):
            continue
    excess = max(0, len(sessions) - keep_runs)
    removed = []
    for directory, manifest in sorted(sessions, key=lambda item: item[0].name):
        if not excess:
            break
        if manifest.get("closed") is not True:
            continue
        try:
            with (directory / ".active.lock").open("r+") as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    continue
                # Recheck after locking: another supervisor may have changed the marker.
                latest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
                if latest.get("owner") != OWNER or latest.get("closed") is not True:
                    continue
                shutil.rmtree(directory)
            removed.append(str(directory))
            excess -= 1
        except (OSError, ValueError, AttributeError):
            continue
    return removed


def stop_process(process: subprocess.Popen, timeout_s: float = 5.0) -> dict:
    """Signal the whole private process group, with bounded escalation waits."""
    escalations = []
    for sig, wait_s in ((signal.SIGINT, timeout_s), (signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
        deadline = time.monotonic() + wait_s
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
        if sig != signal.SIGINT:
            escalations.append(signal.Signals(sig).name)
        try:
            process.wait(timeout=wait_s)
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return {"exit_code": process.returncode, "escalations": escalations}
            # A wrapper can exit before its recorder child. Do not abandon that group.
            while time.monotonic() < deadline:
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    return {"exit_code": process.returncode, "escalations": escalations}
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        except subprocess.TimeoutExpired:
            pass
    return {"exit_code": process.poll(), "escalations": escalations,
            "error": "process group persists after SIGKILL (uninterruptible I/O or descendant zombie)"}


class Recorder:
    """Own the lifetime of one run directory and one mature rosbag subprocess."""

    def __init__(self, config: Config, effective_params: Optional[Mapping[str, Any]] = None,
                 config_namespace: str = "/backtrack_node"):
        self.config = config
        self.params = dict(effective_params or {})
        self.namespace = config_namespace
        self.run_dir = None
        self.manifest = {}
        self.process = None
        self.lock = None
        self.output = None
        self.error = ""
        self.state = "disabled" if not config.enabled else "starting"

    def start(self, popen=subprocess.Popen) -> None:
        if not self.config.enabled:
            return
        try:
            root = self.config.output_dir
            root.mkdir(parents=True, exist_ok=True)
            root = root.resolve()
            if shutil.disk_usage(root).free < self.config.min_space_mb * 1024 * 1024:
                raise OSError("free space is below log_min_space_mb")
            prefix = "run_" + datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ_")
            self.run_dir = Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))
            self.lock = (self.run_dir / ".active.lock").open("w")
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            command = bag_command(self.config, self.run_dir)
            self.manifest = {
                "owner": OWNER, "schema_version": SCHEMA_VERSION,
                "recorder_version": "1", "run_id": self.run_dir.name, "start_utc": utc_now(),
                "start_monotonic_s": time.monotonic(), "closed": False,
                "config_namespace": self.namespace, "effective_parameters": self.params,
                "recorder_config": dataclasses.asdict(self.config), "command": command,
            }
            path = self.params.get("trajectory_config_path", "")
            trajectory_config = {"path": path}
            if isinstance(path, str) and path:
                try:
                    trajectory_config["yaml"] = Path(path).read_text(encoding="utf-8")
                except (OSError, UnicodeError) as error:
                    trajectory_config["read_error"] = str(error)
            self.manifest["trajectory_config"] = trajectory_config
            write_manifest(self.run_dir / "manifest.json", self.manifest)
            prune_closed_runs(root, self.config.keep_runs)
            self.output = (self.run_dir / "recorder.log").open("ab", buffering=0)
            self.process = popen(command, stdin=subprocess.DEVNULL, stdout=self.output,
                                 stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
            self.manifest["pid"] = self.process.pid
            write_manifest(self.run_dir / "manifest.json", self.manifest)
            self.state = "recording"
        except Exception as error:
            self.error = str(error)
            self.state = "failed"
            self.close("startup_failed")

    def attach_configuration(self, configuration: Any) -> None:
        if self.run_dir and not self.manifest.get("closed"):
            self.manifest["runtime_configuration"] = configuration
            history = self.manifest.setdefault("runtime_configurations", [])
            instance_id = configuration.get("node_instance_id") if isinstance(configuration, dict) else None
            existing = next((index for index, entry in enumerate(history)
                             if (instance_id and isinstance(entry, dict) and
                                 entry.get("node_instance_id") == instance_id) or
                             (not instance_id and entry == configuration)), None)
            if existing is None:
                history.append(configuration)
            else:
                history[existing] = configuration
            try:
                write_manifest(self.run_dir / "manifest.json", self.manifest)
            except OSError as error:
                self.error = "manifest update failed: " + str(error)

    def poll(self) -> None:
        if self.process and self.process.poll() is not None and not self.manifest.get("closed"):
            self.error = "rosbag exited unexpectedly with code " + str(self.process.returncode)
            self.state = "failed"
            self.close("recorder_exited")

    def status(self) -> dict:
        state = self.state
        free_bytes = None
        try:
            free_bytes = shutil.disk_usage(self.config.output_dir).free
            if state == "recording" and free_bytes < self.config.min_space_mb * 1024 * 1024:
                state = "insufficient_space"
        except OSError:
            pass
        return {"schema_version": SCHEMA_VERSION, "enabled": self.config.enabled,
                "state": state, "run_id": self.run_dir.name if self.run_dir else "",
                "output_dir": str(self.run_dir or self.config.output_dir),
                "pid": self.process.pid if self.process else None,
                "exit_code": self.process.returncode if self.process else None,
                "error": self.error, "free_bytes": free_bytes,
                "monotonic_s": time.monotonic(), "utc": utc_now()}

    def close(self, reason: str = "shutdown") -> None:
        if self.manifest.get("closed"):
            return
        result = {"exit_code": None, "escalations": []}
        if self.process:
            result = stop_process(self.process)
        if result.get("error"):
            self.error = result["error"]
            self.state = "failed"
        if self.run_dir and self.manifest:
            active_bags = [path.name for path in self.run_dir.glob("*.active")]
            self.manifest.update({"closed": not bool(result.get("error")), "end_utc": utc_now(),
                                  "end_monotonic_s": time.monotonic(), "stop_reason": reason,
                                  "interrupted": reason != "shutdown" or bool(result["escalations"]) or
                                  bool(active_bags) or result["exit_code"] not in (None, 0),
                                  "child_result": result, "error": self.error,
                                  "active_bag_files": active_bags})
            try:
                write_manifest(self.run_dir / "manifest.json", self.manifest)
            except OSError as error:
                self.error = "manifest close failed: " + str(error)
        if self.output:
            self.output.close()
            self.output = None
        if self.lock:
            self.lock.close()
            self.lock = None
        if self.state == "recording":
            self.state = "closed"


def main() -> int:
    # ROS is imported only when invoked by roslaunch, never by host unit tests.
    import rospy
    from std_msgs.msg import String

    rospy.init_node("backtrack_log_recorder")
    publisher = rospy.Publisher("/px4_backtrack/log/recorder_status", String, queue_size=1, latch=True)
    stop = threading.Event()
    rospy.on_shutdown(stop.set)
    try:
        config, namespace, params = Config.load(rospy.get_param)
        recorder = Recorder(config, params, namespace)
    except Exception as error:
        rospy.logerr("[FlightLog] Invalid recording configuration: %s", error)
        while not rospy.is_shutdown():
            publisher.publish(String(data=json.dumps({"state": "failed", "error": str(error)})))
            stop.wait(1.0)
        return 1
    latest_configuration = [None]

    def configuration_received(message):
        try:
            latest_configuration[0] = json.loads(message.data)
        except ValueError:
            latest_configuration[0] = {"raw": message.data}

    last_configuration = None
    last_health = None
    subscriber = None
    try:
        recorder.start()
        subscriber = rospy.Subscriber("/px4_backtrack/log/configuration", String,
                                      configuration_received, queue_size=1)
        if recorder.error:
            rospy.logerr("[FlightLog] Recording unavailable: %s", recorder.error)
        elif config.enabled:
            rospy.loginfo("[FlightLog] Recording to %s", recorder.run_dir)
        while not rospy.is_shutdown():
            if latest_configuration[0] is not None:
                configuration = latest_configuration[0]
                if configuration != last_configuration:
                    recorder.attach_configuration(configuration)
                    last_configuration = configuration
            recorder.poll()
            status = recorder.status()
            health = (status["state"], status["error"])
            if health != last_health:
                if status["state"] == "failed":
                    rospy.logerr("[FlightLog] Recorder failed: %s", status["error"])
                elif status["state"] == "insufficient_space":
                    rospy.logwarn("[FlightLog] Insufficient disk space; recording may be suspended")
                last_health = health
            publisher.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
            stop.wait(1.0)  # Wall/monotonic wait; simulation /clock can be paused.
    except rospy.ROSInterruptException:
        pass
    finally:
        recorder.close()
        if subscriber:
            subscriber.unregister()
    return 0 if not recorder.error else 1


if __name__ == "__main__":
    raise SystemExit(main())
