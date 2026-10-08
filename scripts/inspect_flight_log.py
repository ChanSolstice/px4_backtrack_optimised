#!/usr/bin/env python3
"""Inspect backtrack rosbag evidence without modifying the recorded bags.

Usage (inside a ROS Noetic environment)::

    python3 inspect_flight_log.py /absolute/log/root/run_20261008_...
    python3 inspect_flight_log.py flight_0.bag flight_1.bag --output-dir /tmp/report

The reports describe received samples, not a complete flight safety certificate.
Only bag reading needs ROS; the aggregation functions use the Python standard library.
"""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import re
import sys


CONTROL_TOPIC = "/px4_backtrack/log/control"
PLANNING_TOPIC = "/px4_backtrack/log/planning"
CONFIGURATION_TOPIC = "/px4_backtrack/log/configuration"
RECORDER_STATUS_TOPIC = "/px4_backtrack/log/recorder_status"
SETPOINT_TOPIC = "/mavros/setpoint_raw/local"
ROSOUT_TOPICS = ("/rosout", "/rosout_agg")
LATEST_CONTROL_CONTEXT = object()


def finite_number(value):
    """Return a finite numeric value, or None; never replace invalid data with zero."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value if math.isfinite(value) else None


def json_value(value, nonfinite, path=""):
    """Serialize dynamic ROS messages; record the location and kind of NaN/Inf."""
    if isinstance(value, float) and not math.isfinite(value):
        nonfinite[path] = "NaN" if math.isnan(value) else (
            "+Infinity" if value > 0 else "-Infinity")
        return None
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, (list, tuple)):
        return [json_value(item, nonfinite, "{}[{}]".format(path, i))
                for i, item in enumerate(value)]
    if isinstance(value, dict):
        fields = value.items()
    elif hasattr(value, "__slots__"):
        fields = ((name, getattr(value, name)) for name in value.__slots__)
    elif hasattr(value, "__dict__"):
        fields = vars(value).items()
    else:
        raise TypeError("Unsupported recorded value at {}: {}".format(path, type(value).__name__))
    return {str(name): json_value(item, nonfinite, "{}.{}".format(path, name).lstrip("."))
            for name, item in fields if not str(name).startswith("_")}


class NumericStats:
    def __init__(self):
        self.count = self.invalid = 0
        self.total = 0.0
        self.minimum = self.maximum = None

    def add(self, value):
        value = finite_number(value)
        if value is None:
            self.invalid += 1
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def report(self):
        return {"samples": self.count, "invalid_samples": self.invalid,
                "min": self.minimum, "max": self.maximum,
                "mean": self.total / self.count if self.count else None}


class FlightLogInspector:
    """Aggregate (topic, message, bag_time_seconds) records, including dynamic messages."""

    def __init__(self):
        self.topic_counts = Counter()
        self.issues = []
        self.events = []
        self.flights = {}
        self.plans = {}
        self.plan_metrics = {}
        self.planning_inputs = {}
        self.configurations = []
        self.recorder_states = []
        self.recorder_runs = {}
        self.previous_control = None
        self.current_node_instance_id = None
        self.sequences = {}
        self.sequence_missing = self.sequence_resets = self.sequence_duplicates = 0
        self.missing_fields = set()
        self.first_bag_time = self.last_bag_time = None
        self.previous_bag_time = None
        self.ros_warning_count = self.ros_error_count = 0

    def issue(self, code, detail, severity="warning"):
        item = {"severity": severity, "code": code, "detail": detail}
        if item not in self.issues:
            self.issues.append(item)

    def field(self, message, name, topic):
        if hasattr(message, name):
            return getattr(message, name)
        key = (topic, name)
        if key not in self.missing_fields:
            self.missing_fields.add(key)
            self.issue("missing_field", "{} lacks field '{}'; affected statistics are unavailable.".format(
                topic, name))
        return None

    def event(self, timestamp, category, text, flight=None, plan=None, node=LATEST_CONTROL_CONTEXT):
        self.events.append({"bag_time_s": finite_number(timestamp), "category": category,
                            "node_instance_id": self.current_node_instance_id if node is LATEST_CONTROL_CONTEXT else node,
                            "flight_id": flight, "plan_id": plan, "message": str(text)})

    def metric(self, flight, name, value):
        flight["metrics"].setdefault(name, NumericStats()).add(value)
        if flight["active_plan_id"]:
            key = "{}:{}:{}".format(flight["node_instance_id"], flight["flight_id"], flight["active_plan_id"])
            self.plan_metrics.setdefault(key, {}).setdefault(name, NumericStats()).add(value)

    def consume(self, topic, message, timestamp):
        timestamp = finite_number(timestamp)
        self.topic_counts[topic] += 1
        if timestamp is not None:
            self.first_bag_time = timestamp if self.first_bag_time is None else min(
                self.first_bag_time, timestamp)
            self.last_bag_time = timestamp if self.last_bag_time is None else max(
                self.last_bag_time, timestamp)
            if self.previous_bag_time is not None and timestamp < self.previous_bag_time:
                self.issue("out_of_order_records", "Bag records overlap or go backwards in time; "
                           "event ordering and adjacent-sample statistics require inspection.")
            self.previous_bag_time = timestamp
        if topic == CONTROL_TOPIC:
            self.consume_control(message, timestamp)
        elif topic == PLANNING_TOPIC:
            self.consume_planning(message, timestamp)
        elif topic == CONFIGURATION_TOPIC:
            self.consume_configuration(message)
        elif topic == RECORDER_STATUS_TOPIC:
            self.consume_recorder_status(message, timestamp)
        elif topic == SETPOINT_TOPIC:
            self.consume_setpoint(message)
        elif topic in ROSOUT_TOPICS:
            self.consume_rosout(message, timestamp)

    def consume_control(self, message, timestamp):
        names = ("node_instance_id", "flight_id", "plan_id", "steady_time", "tick_interval", "tick_cost",
                 "current_state", "fcu_mode", "armed", "inputs_fresh", "odom_valid",
                 "localization_reset", "tick_clock_reset", "odom_receipt_age", "state_receipt_age",
                 "actual_velocity", "projection_valid", "projected_s", "target_s", "cross_track_error",
                 "trajectory_valid", "reference_time", "clock_rate", "fallback_valid",
                 "fallback_index", "fallback_count", "route_projection_valid", "route_distance",
                 "route_prediction_valid", "route_predicted_deviation", "route_safety_hold", "input_hold",
                 "yaw_alignment_timeout", "auto_land_yield", "last_trigger", "last_trigger_reason",
                 "input_failure_reason", "odometry_rejection_reason", "rc_loss", "low_battery",
                 "operation", "operation_sequence", "operation_steady_time",
                 "recorded_points", "planning_requested", "planning_inflight", "mode_request_present",
                 "mode_request", "tick_sequence")
        values = {name: self.field(message, name, CONTROL_TOPIC) for name in names}
        node = values["node_instance_id"]
        flight_id, plan_id = values["flight_id"], values["plan_id"]
        if not node or flight_id is None:
            self.issue("unidentifiable_control", "Control records lack node/flight identity; "
                       "they are not merged into guessed flight groups.")
            self.previous_control = None
            return
        self.current_node_instance_id = node
        flight_key = "{}:{}".format(node, flight_id)
        flight = self.flights.setdefault(flight_key, {
            "node_instance_id": node, "flight_id": flight_id, "control_samples": 0, "setpoint_samples": 0,
            "first_bag_time_s": timestamp, "last_bag_time_s": timestamp,
            "states": Counter(), "setpoint_type_masks": Counter(), "metrics": {},
            "active_plan_id": plan_id,
            "invalid_input_samples": 0, "route_safety_hold_samples": 0,
            "input_hold_samples": 0, "last_control": {}, "previous": None})
        flight["control_samples"] += 1
        flight["active_plan_id"] = plan_id
        if plan_id:
            key = "{}:{}:{}".format(node, flight_id, plan_id)
            plan = self.plans.setdefault(key, {"node_instance_id": node, "flight_id": flight_id, "plan_id": plan_id,
                                               "phases": [], "input_available": False})
            plan["control_samples"] = plan.get("control_samples", 0) + 1
        flight["last_bag_time_s"] = timestamp
        flight["states"][str(values["current_state"])] += 1
        for field, metric_name in (("tick_interval", "tick_interval_s"), ("tick_cost", "tick_cost_s"),
                                   ("odom_receipt_age", "odom_age_s"), ("state_receipt_age", "state_age_s")):
            self.metric(flight, metric_name, values[field])
        if values["odom_valid"] is True:
            velocity = values["actual_velocity"]
            components = [self.field(velocity, axis, CONTROL_TOPIC + ".actual_velocity")
                          for axis in ("x", "y", "z")]
            speed = math.hypot(*components) if all(
                finite_number(x) is not None for x in components) else None
            self.metric(flight, "actual_speed_m_s", speed)
        if values["projection_valid"] is True:
            self.metric(flight, "cross_track_error_m", values["cross_track_error"])
            projected = finite_number(values["projected_s"])
            target = finite_number(values["target_s"])
            self.metric(flight, "projected_s_m", projected)
            self.metric(flight, "target_s_m", target)
            self.metric(flight, "reference_progress_lead_m",
                        target - projected if target is not None and projected is not None else None)
        if values["route_projection_valid"] is True:
            self.metric(flight, "route_distance_m", values["route_distance"])
        if values["route_prediction_valid"] is True:
            self.metric(flight, "predicted_deviation_m", values["route_predicted_deviation"])
        if values["trajectory_valid"] is True:
            self.metric(flight, "reference_time_s", values["reference_time"])
            self.metric(flight, "clock_rate", values["clock_rate"])
        for flag, counter in (("inputs_fresh", "invalid_input_samples"),
                              ("route_safety_hold", "route_safety_hold_samples"),
                              ("input_hold", "input_hold_samples")):
            if values[flag] is (False if flag == "inputs_fresh" else True):
                flight[counter] += 1
        previous = flight["previous"]
        transition_fields = ("current_state", "fcu_mode", "armed", "inputs_fresh", "odom_valid",
                             "localization_reset", "tick_clock_reset", "route_safety_hold", "input_hold",
                             "yaw_alignment_timeout", "auto_land_yield", "last_trigger", "last_trigger_reason",
                             "input_failure_reason", "odometry_rejection_reason", "rc_loss", "low_battery",
                             "fallback_valid", "planning_requested", "planning_inflight")
        for name in transition_fields:
            value = values[name]
            if value is not None and (previous is None or previous[name] != value):
                if previous is None and not value and name not in ("current_state", "fcu_mode", "armed"):
                    continue
                before = "initial" if previous is None else str(previous[name])
                self.event(timestamp, name, "{} -> {}".format(before, value), flight_id, plan_id)
        operation = values["operation"]
        if operation and (previous is None or
                          values["operation_sequence"] != previous["operation_sequence"] or
                          (values["operation_sequence"] is None and operation != previous["operation"])):
            self.event(timestamp, "operation", "{} (steady_time_s={})".format(
                operation, values["operation_steady_time"]), flight_id, plan_id)
        if values["mode_request_present"] is True and (previous is None or
                previous["mode_request_present"] is not True or
                previous["mode_request"] != values["mode_request"]):
            self.event(timestamp, "mode_intent", "Mode requested: {}".format(
                values["mode_request"]), flight_id, plan_id)
        if previous is not None:
            now = finite_number(values["steady_time"])
            before = finite_number(previous["steady_time"])
            if now is not None and before is not None and now >= before:
                self.metric(flight, "observed_diagnostic_interval_s", now - before)
            elif now is not None and before is not None:
                self.issue("steady_clock_reset", "Flight {} steady time decreased; "
                           "node restart or mixed recordings may be present.".format(flight_id))
        sequence = values["tick_sequence"]
        if isinstance(sequence, int) and not isinstance(sequence, bool):
            sequence_info = self.sequences.setdefault(node, {"first": sequence, "last": None,
                                        "missing_between_samples": 0, "resets": 0, "duplicates": 0})
            previous_sequence = sequence_info["last"]
            if previous_sequence is not None:
                delta = (sequence - previous_sequence) % (2 ** 64)
                if delta == 0:
                    self.sequence_duplicates += 1
                    sequence_info["duplicates"] += 1
                elif delta < 2 ** 63:
                    self.sequence_missing += delta - 1
                    sequence_info["missing_between_samples"] += delta - 1
                    if delta > 1:
                        self.event(timestamp, "diagnostic_sequence_gap", "{} missing sequence(s) "
                                   "between {} and {}".format(delta - 1, previous_sequence,
                                                              sequence), flight_id, plan_id)
                else:
                    self.sequence_resets += 1
                    sequence_info["resets"] += 1
                    self.event(timestamp, "diagnostic_sequence_reset", "{} -> {}".format(
                        previous_sequence, sequence), flight_id, plan_id)
            elif sequence > 1:
                self.issue("missing_diagnostic_prefix", "First control sequence is {}; earlier "
                           "evidence may have been retained elsewhere, deleted, or not recorded.".format(
                               sequence))
            sequence_info["last"] = sequence
        flight["last_control"] = {name: values[name] for name in (
            "current_state", "fcu_mode", "armed", "fallback_valid", "fallback_index", "fallback_count",
            "route_safety_hold", "input_hold", "last_trigger", "plan_id", "recorded_points")}
        flight["previous"] = values
        self.previous_control = values

    def consume_planning(self, message, timestamp):
        node = self.field(message, "node_instance_id", PLANNING_TOPIC)
        flight_id = self.field(message, "flight_id", PLANNING_TOPIC)
        plan_id = self.field(message, "plan_id", PLANNING_TOPIC)
        phase = self.field(message, "phase", PLANNING_TOPIC)
        if not node or flight_id is None or plan_id is None:
            self.issue("unidentifiable_planning", "Planning records lack node/flight/plan identity; "
                       "retrieve their original bag rather than relying on guessed associations.")
            return
        key = "{}:{}:{}".format(node, flight_id, plan_id)
        record = self.plans.setdefault(key, {"node_instance_id": node, "flight_id": flight_id, "plan_id": plan_id,
                                          "phases": [], "input_available": False})
        record["phases"].append(phase)
        nonfinite = {}
        snapshot = json_value(message, nonfinite)
        input_names = ("raw_position", "raw_yaw_rad", "raw_timestamp_s", "effective_config")
        for name in input_names:
            self.field(message, name, PLANNING_TOPIC)
        previous_input = self.planning_inputs.get(key)
        input_nonfinite = {name: kind for name, kind in nonfinite.items()
                           if name.startswith(input_names[:3])}
        previous_nonfinite = ({name: kind for name, kind in previous_input["nonfinite_fields"].items()
                               if name.startswith(input_names[:3])} if previous_input else {})
        if previous_input and (input_nonfinite != previous_nonfinite or any(
                previous_input["planning_trace"].get(name) != snapshot.get(name) for name in input_names)):
            self.issue("conflicting_planning_inputs", "Plan {} repeats an identity with different frozen "
                       "inputs. The first input file is kept; alternatives are retained in summary.json.".format(key))
            alternative = {"bag_time_s": timestamp, "planning_trace": snapshot,
                           "nonfinite_fields": nonfinite}
            alternatives = record.setdefault("conflicting_inputs", [])
            if alternative not in alternatives:
                alternatives.append(alternative)
        # Every phase retains the frozen input, including latched finished/discarded traces.
        if all(hasattr(message, name) for name in input_names) and (
                previous_input is None or (phase == "started" and "conflicting_inputs" not in record)):
            self.planning_inputs[key] = {"bag_time_s": timestamp, "input_source_phase": phase,
                                         "planning_trace": snapshot, "nonfinite_fields": nonfinite}
            record["input_available"] = True
            record["raw_point_count"] = len(snapshot.get("raw_position", []))
            lengths = [len(snapshot[name]) for name in input_names[:3]]
            if len(set(lengths)) != 1:
                self.issue("planning_input_length_mismatch", "Plan {} input array lengths differ: {}.".format(
                    key, lengths))
            if nonfinite:
                self.issue("nonfinite_planning_values", "Plan {} contains nonfinite values; exact field "
                           "locations and NaN/Infinity kinds are retained in its input JSON.".format(key))
        if phase == "started":
            self.event(timestamp, "planning_started", "Frozen input: {} points".format(
                record.get("raw_point_count", 0)), flight_id, plan_id, node)
        elif phase in ("finished", "discarded"):
            for name in ("success", "status", "message", "fallback_ready", "generation_cost_ms",
                         "duration_s", "path_length_m", "max_speed_m_s", "max_yaw_rate_rad_s",
                         "motion_speed_scale", "motion_limits_verified", "corridor_verified"):
                self.field(message, name, PLANNING_TOPIC)
            record["result"] = {name: value for name, value in snapshot.items()
                                if name not in ("raw_position", "raw_yaw_rad", "raw_timestamp_s",
                                                "effective_config", "header")}
            record["nonfinite_result_fields"] = nonfinite
            self.event(timestamp, "planning_" + phase, "status={} success={} fallback_ready={} {}".format(
                snapshot.get("status"), snapshot.get("success"), snapshot.get("fallback_ready"),
                snapshot.get("message", "")), flight_id, plan_id, node)
        else:
            self.issue("unknown_planning_phase", "Plan {} has phase {!r}.".format(key, phase))

    def consume_configuration(self, message):
        text = self.field(message, "data", CONFIGURATION_TOPIC)
        if text is None:
            return
        try:
            configuration = json.loads(text)
        except (ValueError, TypeError) as exc:
            self.issue("invalid_configuration_json", str(exc))
            configuration = {"raw_text": text}
        if configuration not in self.configurations:
            self.configurations.append(configuration)

    def consume_setpoint(self, message):
        if self.previous_control is None:
            self.issue("unassigned_setpoints", "Some setpoints precede the first control diagnostic; "
                       "they cannot be assigned to a flight from this recording.")
            return
        key = "{}:{}".format(self.previous_control["node_instance_id"], self.previous_control["flight_id"])
        flight = self.flights[key]
        flight["setpoint_samples"] += 1
        mask = self.field(message, "type_mask", SETPOINT_TOPIC)
        flight["setpoint_type_masks"][str(mask)] += 1
        # MAVROS PositionTarget bits 3..5 mean that the corresponding velocity is ignored.
        if isinstance(mask, int) and mask & (7 << 3) == 0:
            velocity = self.field(message, "velocity", SETPOINT_TOPIC)
            components = [self.field(velocity, axis, SETPOINT_TOPIC + ".velocity")
                          for axis in ("x", "y", "z")]
            speed = math.hypot(*components) if all(
                finite_number(x) is not None for x in components) else None
            self.metric(flight, "published_velocity_feedforward_m_s", speed)

    def consume_recorder_status(self, message, timestamp):
        text = self.field(message, "data", RECORDER_STATUS_TOPIC)
        try:
            status = json.loads(text)
            if not isinstance(status, dict):
                raise ValueError("Recorder status is not a JSON object")
        except (ValueError, TypeError) as exc:
            self.issue("invalid_recorder_status", str(exc))
            return
        key = status.get("run_id") or status.get("output_dir") or "unidentified"
        run = self.recorder_runs.setdefault(key, {"samples": 0, "latest": {}, "free_bytes": NumericStats()})
        previous = run["latest"]
        run["samples"] += 1
        run["free_bytes"].add(status.get("free_bytes"))
        run["latest"] = status
        if not previous or any(previous.get(name) != status.get(name)
                               for name in ("state", "error", "output_dir", "pid", "exit_code")):
            self.recorder_states.append({"bag_time_s": timestamp, "status": status})
            self.event(timestamp, "recorder_status", "run={} state={} dir={} error={}".format(
                status.get("run_id", ""), status.get("state"), status.get("output_dir", ""),
                status.get("error", "")), node=None)
        if status.get("state") in ("failed", "insufficient_space") or status.get("error"):
            self.issue("recorder_unhealthy", "run={} state={} error={}; flight evidence may be "
                       "incomplete, and a dropped-record count cannot be inferred from this warning.".format(
                           status.get("run_id", ""), status.get("state"), status.get("error", "")))

    def consume_rosout(self, message, timestamp):
        level = self.field(message, "level", "/rosout")
        text = self.field(message, "msg", "/rosout")
        if text is None or level is None:
            return
        if level >= 8:
            self.ros_error_count += 1
        elif level >= 4:
            self.ros_warning_count += 1
        mode_event = "[SetModeClient]" in text
        if level >= 4 or mode_event:
            name = self.field(message, "name", "/rosout")
            values = self.previous_control or {}
            recorder_source = bool(name and (name.rsplit("/", 1)[-1].startswith("record_") or
                                   name.rsplit("/", 1)[-1] == "record" or "rosbag" in name or
                                   "flight_log" in name or name.rsplit("/", 1)[-1] == "backtrack_log_recorder"))
            self.event(timestamp, "mode_service" if mode_event else "ros_error" if level >= 8
                       else "ros_warning", "{}: {}".format(name, text),
                       None if recorder_source else values.get("flight_id"),
                       None if recorder_source else values.get("plan_id"),
                       None if recorder_source else self.current_node_instance_id)
            self.events[-1]["source"] = name

    def report(self):
        if not self.topic_counts[CONTROL_TOPIC]:
            self.issue("missing_control_diagnostics", "No typed control diagnostics were read; "
                       "flight progress, holds and sample statistics are unavailable.")
        if not self.topic_counts[CONFIGURATION_TOPIC]:
            self.issue("missing_configuration", "No startup configuration was read. Retention may "
                       "have removed the first bag; retrieve the earlier chunks if available.")
        if self.sequence_missing:
            self.issue("diagnostic_sequence_gaps", "{} control sequence(s) are missing between "
                       "received samples; inspect recorder.log, transport queues and bag retention.".format(
                           self.sequence_missing))
        if self.sequence_resets or self.sequence_duplicates:
            self.issue("diagnostic_sequence_restarts", "Sequence resets={} duplicates={}; node "
                       "restarts or overlapping bags may be present.".format(
                           self.sequence_resets, self.sequence_duplicates))
        for key, plan in self.plans.items():
            if not plan["input_available"]:
                self.issue("missing_planning_input", "Plan {} has no observed frozen input trace. It may "
                           "have been cancelled before the worker started, or earlier evidence may "
                           "be absent. Check control operations and earlier chunks; this warning "
                           "alone does not establish recording loss.".format(key))
            elif not any(phase in ("finished", "discarded") for phase in plan["phases"]):
                self.issue("unfinished_planning_trace", "Plan {} has no observed finish/discard; "
                           "planning may still have been running, or the evidence may end early.".format(key))
        flights = {}
        for key, flight in self.flights.items():
            flights[key] = {name: value for name, value in flight.items()
                            if name not in ("previous", "metrics", "states", "setpoint_type_masks", "active_plan_id")}
            flights[key]["states"] = dict(flight["states"])
            flights[key]["setpoint_type_masks"] = dict(flight["setpoint_type_masks"])
            flights[key]["metrics"] = {name: stats.report()
                                       for name, stats in flight["metrics"].items()}
        plans = {key: dict(plan) for key, plan in self.plans.items()}
        for key, metrics in self.plan_metrics.items():
            plans[key]["control_metrics"] = {name: stats.report() for name, stats in metrics.items()}
        return {"schema_version": 1,
                "interpretation": [
                    "Statistics cover received log samples only; they do not certify a complete flight.",
                    "route_distance_m measures distance to the original recorded route; "
                    "cross_track_error_m measures distance to the planned trajectory.",
                    "reference_progress_lead_m = target_s - projected_s; positive means the reference leads.",
                    "Setpoints are grouped using the latest received control diagnostic.",
                    "ROS text events use the latest control context; this association is approximate, "
                    "while the original ROS message text is retained.",
                    "Null means invalid, nonfinite, or unavailable; invalid samples are never replaced with zero."],
                "first_bag_time_s": self.first_bag_time, "last_bag_time_s": self.last_bag_time,
                "topic_counts": dict(self.topic_counts),
                "control_sequence": {"missing_between_samples": self.sequence_missing,
                                     "resets": self.sequence_resets, "duplicates": self.sequence_duplicates,
                                     "by_node_instance": self.sequences},
                "ros_warnings": self.ros_warning_count, "ros_errors": self.ros_error_count,
                "recorder_states": self.recorder_states,
                "recorder_runs": {key: {"samples": run["samples"], "latest": run["latest"],
                                        "free_bytes": run["free_bytes"].report()}
                                  for key, run in self.recorder_runs.items()},
                "flights": flights, "plans": plans, "configurations": self.configurations,
                "issues": list(self.issues)}


def discover_inputs(paths):
    """Return completed bags, optional run directory, and visible evidence problems."""
    bags, active, issues = set(), set(), []
    run_directory = paths[0] if len(paths) == 1 and paths[0].is_dir() else None
    for path in paths:
        if path.is_dir():
            bags.update(item.resolve() for item in path.glob("*.bag") if item.is_file())
            active.update(item.resolve() for item in path.glob("*.bag.active") if item.is_file())
        elif path.is_file() and path.name.endswith(".bag.active"):
            active.add(path.resolve())
        elif path.is_file() and path.suffix == ".bag":
            bags.add(path.resolve())
        else:
            issues.append({"severity": "error", "code": "missing_input",
                           "detail": "Input is absent or is not a bag/run directory: {}".format(path)})
    for path in sorted(active):
        issues.append({"severity": "error", "code": "unfinished_bag",
                       "detail": "{} is unfinished and was not read. Preserve the original, copy it "
                                 "elsewhere, then use ROS rosbag reindex on the copy if recovery is needed. "
                                 "Recovery cannot reconstruct records that were never written.".format(path)})
    groups = {}
    for path in sorted(bags):
        match = re.match(r"^(.*)_(\d+)\.bag$", path.name)
        if match:
            groups.setdefault((str(path.parent), match.group(1)), []).append(int(match.group(2)))
    for (_, prefix), indices in groups.items():
        indices.sort()
        if indices[0] != 0:
            issues.append({"severity": "warning", "code": "missing_bag_prefix",
                           "detail": "{} begins at split {}; earlier chunks may have been deleted by "
                                     "retention or omitted from the supplied inputs.".format(prefix, indices[0])})
        gaps = [(left + 1, right - 1) for left, right in zip(indices, indices[1:]) if right > left + 1]
        if gaps:
            issues.append({"severity": "warning", "code": "missing_bag_chunks",
                           "detail": "{} has missing split ranges {}. Retrieve those chunks if possible; "
                                     "the report cannot reconstruct their contents.".format(prefix, gaps)})
    if not bags:
        issues.append({"severity": "error", "code": "no_completed_bags",
                       "detail": "No completed .bag files were found."})
    return sorted(bags), run_directory, issues


def write_reports(inspector, output_dir, evidence=None):
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = inspector.report()
    summary["evidence"] = evidence or {}
    for key, snapshot in inspector.planning_inputs.items():
        filename = "planning_{}.json".format(re.sub(r"[^A-Za-z0-9_.-]", "_", key))
        write_json(output_dir / filename, snapshot)
        summary["plans"][key]["input_file"] = filename
    write_json(output_dir / "summary.json", summary)
    with (output_dir / "events.log").open("w", encoding="utf-8") as stream:
        stream.write("# Received log events; bag_time_s is ROS/bag time, not the steady control clock.\n")
        for event in inspector.events:
            timestamp = event["bag_time_s"]
            stamp = "unknown" if timestamp is None else "{:.6f}".format(timestamp)
            text = event["message"].replace("\r", "\\r").replace("\n", "\\n")
            stream.write("t={} node={} flight={} plan={} source={} [{}] {}\n".format(
                stamp, event["node_instance_id"], event["flight_id"], event["plan_id"],
                event.get("source", "diagnostics"), event["category"], text))
        for issue in summary["issues"]:
            stream.write("[{}:{}] {}\n".format(issue["severity"], issue["code"], issue["detail"]))
    return summary


def order_bag_chunks(metadata):
    """Keep a split series in file order even when ROS time freezes or resets.

    Each item is (first ROS timestamp, path, size). Order independent series by
    the first available split's time; within a series use the numeric index.
    """
    groups = {}
    for item in metadata:
        _, path, _ = item
        match = re.match(r"^(.*)_(\d+)\.bag$", path.name)
        key = (str(path.parent), match.group(1) if match else path.name)
        groups.setdefault(key, []).append((int(match.group(2)) if match else 0, item))
    series = []
    for key, chunks in groups.items():
        chunks.sort(key=lambda indexed: indexed[0])
        series.append((chunks[0][1][0], key, [item for _, item in chunks]))
    return [item for _, _, chunks in sorted(series) for item in chunks]


def write_json(path, data):
    nonfinite = {}
    clean = json_value(data, nonfinite)
    if nonfinite:
        clean["nonfinite_report_fields"] = nonfinite
    with path.open("w", encoding="utf-8") as stream:
        json.dump(clean, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def read_run_metadata(run_directory, inspector):
    evidence = {}
    if run_directory is None:
        return evidence
    manifest = run_directory / "manifest.json"
    if manifest.exists():
        try:
            evidence["manifest"] = json.loads(manifest.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            inspector.issue("unreadable_manifest", "{}: {}".format(manifest, exc))
    else:
        inspector.issue("missing_manifest", "No manifest.json exists in the supplied run directory.")
    recorder_log = run_directory / "recorder.log"
    if recorder_log.exists():
        try:
            warnings = deque(maxlen=200)
            tail = deque(maxlen=40)
            warning_count = 0
            pattern = re.compile(r"\b(?:warn(?:ing)?|error|fatal)\b|buffer.*(?:exceed|overflow)|"
                                 r"insufficient.*(?:disk|space)|disk.*full|not recording", re.IGNORECASE)
            with recorder_log.open(encoding="utf-8", errors="replace") as stream:
                for number, line in enumerate(stream, 1):
                    line = line.rstrip("\n")
                    tail.append(line)
                    if pattern.search(line):
                        warning_count += 1
                        warnings.append({"line": number, "text": line})
            evidence["recorder_log_tail"] = list(tail)
            evidence["recorder_warning_lines"] = list(warnings)
            evidence["recorder_warning_line_count"] = warning_count
            if warning_count:
                inspector.issue("recorder_log_warnings", "{} warning/error/capacity lines in recorder.log "
                                "(the last 200 are retained). These may indicate lost recording evidence; "
                                "they do not establish the number of records lost.".format(warning_count))
        except OSError as exc:
            inspector.issue("unreadable_recorder_log", str(exc))
    return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", type=Path, help="Run directory or completed bag files")
    parser.add_argument("--output-dir", type=Path, help="Report directory; defaults to the run directory "
                        "or a sibling <first_bag>_inspection directory")
    args = parser.parse_args(argv)
    bags, run_directory, input_issues = discover_inputs(args.inputs)
    output_dir = args.output_dir or run_directory or (
        args.inputs[0].parent / (args.inputs[0].stem + "_inspection"))
    inspector = FlightLogInspector()
    inspector.issues.extend(input_issues)
    evidence = read_run_metadata(run_directory, inspector)
    evidence["requested_bags"] = [str(path) for path in bags]
    evidence["bags"] = []
    # Dynamic rosbag deserialization uses the message definitions embedded in each bag.
    # No import of this package's generated messages is required for offline inspection.
    if bags:
        try:
            import rosbag
        except ImportError:
            inspector.issue("rosbag_unavailable", "Python rosbag is unavailable. Run this script in "
                            "ROS Noetic after sourcing /opt/ros/noetic/setup.bash.", "error")
            bags = []
    bag_order = []
    for path in bags:
        try:
            size = path.stat().st_size
            with rosbag.Bag(str(path), "r") as bag:
                bag_order.append((bag.get_start_time(), path, size))
        except Exception as exc:  # rosbag exposes several I/O, indexing and format exception types.
            inspector.issue("unreadable_bag", "{}: {}: {}. Keep the original; recovery/reindex must "
                            "be attempted on a copy.".format(path, type(exc).__name__, exc), "error")
            evidence["bags"].append({"path": str(path), "read_ok": False})
    for _, path, size in order_bag_chunks(bag_order):
        count = 0
        read_ok = True
        try:
            with rosbag.Bag(str(path), "r") as bag:
                for topic, message, timestamp in bag.read_messages():
                    inspector.consume(topic, message, timestamp.to_sec())
                    count += 1
        except Exception as exc:
            read_ok = False
            inspector.issue("incomplete_bag_read", "{}: {}: {} after {} records. Preserve this "
                            "bag and attempt recovery only on a copy; later bags are still inspected.".format(
                                path, type(exc).__name__, exc, count), "error")
        evidence["bags"].append({"path": str(path), "bytes": size,
                                 "records_read": count, "read_ok": read_ok})
    try:
        summary = write_reports(inspector, output_dir, evidence)
    except OSError as exc:
        print("Cannot write inspection reports: {}".format(exc), file=sys.stderr)
        return 2
    print("Reports: {}".format(output_dir.resolve()))
    print("Flights={} plans={} warnings/errors={} sequence_gaps={}".format(
        len(summary["flights"]), len(summary["plans"]), len(summary["issues"]),
        summary["control_sequence"]["missing_between_samples"]))
    for issue in summary["issues"]:
        print("{}: {}".format(issue["code"], issue["detail"]), file=sys.stderr)
    return 2 if any(item["severity"] == "error" for item in summary["issues"]) else 0


if __name__ == "__main__":
    sys.exit(main())
