import json
import math
import os
from pathlib import Path
import signal
import re
import subprocess
import time
import xmlrpc.client

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11329'
os.environ['ROS_IP'] = '127.0.0.1'

import rospy
import rosbag
from std_msgs.msg import String, Float64
from rosgraph_msgs.msg import Clock
from px4_backtrack_optimised.msg import ControlDiagnostics, PlanningTrace

ROOT = Path('/tmp/log-scale-probe')
OUTPUT = ROOT / ('evidence_' + str(time.time_ns()))
OUTPUT.mkdir()
master_output = (OUTPUT / 'master.log').open('w')
master = subprocess.Popen(['roscore', '-p', '11329'], stdout=master_output,
                          stderr=subprocess.STDOUT, start_new_session=True)
children = []
outputs = []
controls = []
planning = []
costs = []
statuses = []
ready = []

def wait_until(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('probe timed out')

def master_ready():
    try:
        return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1
    except OSError:
        return False

def start(command, label):
    output = (OUTPUT / (label + '.log')).open('w')
    outputs.append(output)
    child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(child)
    return child

def stop(child):
    child.send_signal(signal.SIGINT)
    child.wait(timeout=12)

def raw_input_valid(message):
    assert len(message.raw_position) == len(message.raw_yaw_rad) == len(message.raw_timestamp_s) == 54000
    for i, (point, yaw, timestamp) in enumerate(zip(message.raw_position, message.raw_yaw_rad,
                                                  message.raw_timestamp_s)):
        assert abs(point.x - i * .00005) < 1e-10
        assert point.y == 0 and point.z == 1
        assert abs(yaw - (.123 + i * .000001)) < 1e-10
        assert abs(timestamp - (1000 + i / 15.)) < 1e-9

try:
    wait_until(master_ready)
    xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).setParam('/probe', '/use_sim_time', True)
    rospy.init_node('scale_probe', disable_signals=True)
    (ROOT / 'trajectory.yaml').write_text('v_max: 0.6\na_tan_max: 1.0\naz_up_max: 1.0\naz_down_max: 1.0\n')
    rospy.set_param('/scale_backtrack', {'log_enable': True, 'log_output_dir': str(OUTPUT / 'runs'),
                                       'log_min_space_mb': 1, 'log_split_size_mb': 64,
                                       'log_max_splits': 6, 'log_keep_runs': 1,
                                       'log_topics': ['/probe/blob', '/probe/phase_cost_ms', '/probe/ready'],
                                       'trajectory_config_path': str(ROOT / 'trajectory.yaml')})
    rospy.Subscriber('/px4_backtrack/log/control', ControlDiagnostics,
                     lambda message: controls.append((message.tick_sequence, message.steady_time,
                                                      message.tick_interval, time.monotonic())))
    rospy.Subscriber('/px4_backtrack/log/planning', PlanningTrace,
                     lambda message: planning.append((message.phase, len(message.raw_position))))
    rospy.Subscriber('/probe/phase_cost_ms', Float64, lambda message: costs.append(message.data))
    rospy.Subscriber('/probe/ready', String, lambda message: ready.append(message.data))
    rospy.Subscriber('/px4_backtrack/log/recorder_status', String,
                     lambda message: statuses.append(json.loads(message.data)))
    clock = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
    fixed_clock = Clock()
    fixed_clock.clock = rospy.Time(10)
    clock.publish(fixed_clock)
    command = rospy.Publisher('/probe/command', String, queue_size=1)
    blob = rospy.Publisher('/probe/blob', String, queue_size=1)
    producer = start([str(ROOT / 'publisher')], 'producer')
    recorder = start(['python3', '/tmp/backtrack-log-ws/src/px4_backtrack_optimised/scripts/record_flight_log.py',
                      '__name:=scale_log_recorder', '_config_namespace:=/scale_backtrack'], 'recorder')
    wait_until(lambda: ready and any(status.get('state') == 'recording' for status in statuses))
    wait_until(lambda: blob.get_num_connections() > 0 and command.get_num_connections() > 0)
    time.sleep(.3)
    command.publish(String(data='started'))
    wait_until(lambda: ('started', 54000) in planning)
    command.publish(String(data='finished'))
    wait_until(lambda: ('finished', 54000) in planning and len(costs) == 2)
    start_index = len(controls)
    for _ in range(140):
        # Incompressible extra sensor traffic forces real slice rotation quickly.
        blob.publish(String(data=os.urandom(512 * 1024).hex()))
        clock.publish(fixed_clock)
        time.sleep(.10)
    time.sleep(.5)
    assert rospy.Time.now().to_sec() == 10
    stress_controls = controls[start_index:]
    assert len(stress_controls) >= 70, len(stress_controls)
    stress_gaps = sum(max(0, right[0] - left[0] - 1)
                      for left, right in zip(stress_controls, stress_controls[1:]))
    (OUTPUT / 'live_progress.json').write_text(json.dumps({'planning_publish_cost_ms': costs,
            'control_stress_samples': len(stress_controls), 'missing_live_sequences': stress_gaps,
            'reported_interval_max_s': max(row[2] for row in stress_controls)}) + '\n')
    stop(recorder)
    stop(producer)
    assert recorder.returncode == producer.returncode == 0
    run = next((OUTPUT / 'runs').glob('run_*'))
    manifest = json.loads((run / 'manifest.json').read_text())
    assert manifest['closed'] is True and not manifest['interrupted'], manifest
    assert not list(run.glob('*.active'))
    bags = sorted(run.glob('*.bag'), key=lambda path: int(re.search(r'_(\d+)\.bag$', path.name).group(1)))
    assert len(bags) >= 3, [path.name for path in bags]
    bag_records = []
    for path in bags:
        phases = []
        control_count = 0
        raw_count = 0
        marker_counts = []
        path_counts = []
        with rosbag.Bag(str(path)) as bag:
            for topic, message, stamp in bag.read_messages():
                if topic == '/px4_backtrack/log/planning':
                    raw_input_valid(message)
                    phases.append(message.phase)
                    raw_count = len(message.raw_position)
                if topic == '/px4_backtrack/log/control':
                    control_count += 1
                if topic == '/px4_backtrack/debug/waypoints':
                    assert len(message.markers) == 54003
                    assert len(message.markers[54000].points) == 54000
                    marker_counts.append(len(message.markers))
                if topic == '/px4_backtrack/debug/trajectory':
                    assert len(message.poses) == 20000
                    path_counts.append(len(message.poses))
        bag_records.append({'name': path.name, 'bytes': path.stat().st_size,
                            'planning_phases': phases, 'raw_points_each': raw_count,
                            'control_samples': control_count,
                            'rviz_marker_counts': marker_counts, 'rviz_path_pose_counts': path_counts})
    assert any(row['planning_phases'] == ['finished'] for row in bag_records), bag_records
    assert all(row['rviz_marker_counts'] and row['rviz_path_pose_counts'] for row in bag_records), bag_records
    latest_bag = bags[-1]
    inspector_path = '/tmp/backtrack-log-ws/src/px4_backtrack_optimised/scripts/inspect_flight_log.py'
    inspection = subprocess.run(['python3', inspector_path, str(run), '--output-dir', str(OUTPUT / 'inspection')],
                                capture_output=True, text=True, timeout=60)
    (OUTPUT / 'inspector.log').write_text(inspection.stdout + inspection.stderr)
    assert inspection.returncode == 0, inspection.stderr
    single_inspection = subprocess.run(['python3', inspector_path, str(latest_bag), '--output-dir', str(OUTPUT / 'last_slice_inspection')],
                                       capture_output=True, text=True, timeout=60)
    (OUTPUT / 'last_slice_inspector.log').write_text(single_inspection.stdout + single_inspection.stderr)
    assert single_inspection.returncode == 0, single_inspection.stderr
    for inspection_dir in (OUTPUT / 'inspection', OUTPUT / 'last_slice_inspection'):
        snapshots = list(inspection_dir.glob('planning_*.json'))
        assert len(snapshots) == 1, snapshots
        snapshot = json.loads(snapshots[0].read_text())
        assert len(snapshot['planning_trace']['raw_position']) == 54000, snapshot.keys()
    intervals = sorted(row[2] for row in stress_controls)
    arrivals = sorted(right[3] - left[3] for left, right in zip(stress_controls, stress_controls[1:]))
    result = {'pass': True, 'point_count': 54000, 'represented_recording_duration_s': 53999 / 15.,
              'producer': 'actual C++ DiagnosticsPublisher from built repository library',
              'frozen_ros_time_s': 10.0, 'planning_publish_cost_ms': costs,
              'control_stress': {'samples': len(stress_controls), 'missing_live_sequences': stress_gaps,
                                 'reported_interval_mean_s': sum(intervals) / len(intervals),
                                 'reported_interval_p99_s': intervals[int(.99 * (len(intervals) - 1))],
                                 'reported_interval_max_s': max(intervals),
                                 'arrival_interval_max_s': max(arrivals)},
              'retained_bags': bag_records, 'manifest': str(run / 'manifest.json'),
              'inspector_returncode': inspection.returncode,
              'last_slice_inspector_returncode': single_inspection.returncode,
              'last_slice_restored_points': 54000, 'evidence_directory': str(OUTPUT),
              'limits': ['desktop/container performance, not RK3588', 'synthetic 20Hz control publisher; no flight control algorithm',
                         'compressed time stress test, not one hour continuous operation',
                         'min-space=1MB and six 64MB slices; buffer32MB is production default',
                         'extra incompressible sensor traffic is synthetic; no real SLAM/Gazebo/FCU']}
    (OUTPUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
finally:
    for child in children:
        if child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=2)
    rospy.signal_shutdown('probe complete')
    os.killpg(master.pid, signal.SIGINT)
    master.wait(timeout=10)
    for output in outputs:
        output.close()
    master_output.close()
