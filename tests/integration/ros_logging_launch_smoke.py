#!/usr/bin/env python3
"""Exercise the installed launch on an isolated master; never connect to a FCU."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:{}'.format(port), ROS_HOSTNAME='127.0.0.1')
    processes, streams = [], []

    def start(command, name):
        stream = (output / (name + '.log')).open('w')
        streams.append(stream)
        proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)

    try:
        start(['roscore', '-p', str(port)], 'master')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1:
                    break
            except OSError:
                time.sleep(.05)
        else:
            raise RuntimeError('isolated master did not start')
        import rospy
        from px4_backtrack_optimised.msg import ControlDiagnostics
        from std_msgs.msg import String

        rospy.init_node('logging_launch_smoke', disable_signals=True)
        controls, health = [], []
        rospy.Subscriber('/px4_backtrack/log/control', ControlDiagnostics, controls.append, queue_size=200)
        rospy.Subscriber('/px4_backtrack/log/recorder_status', String,
                         lambda msg: health.append(json.loads(msg.data)), queue_size=20)
        results = {}
        for name, enabled, invalid_config, split_mb in (
            ('enabled', True, False, 64), ('disabled', False, False, 64),
            ('init_failed', True, True, 64), ('bad_split', True, False, 1)
        ):
            controls.clear()
            health.clear()
            directory = output / (name + ' logs')
            root = ET.Element('launch')
            ET.SubElement(root, 'include', file=str(args.launch.resolve()))
            ET.SubElement(root, 'param', name='/backtrack_node/log_output_dir',
                          value=str(directory), type='str')
            ET.SubElement(root, 'param', name='/backtrack_node/log_enable',
                          value=str(enabled).lower(), type='bool')
            ET.SubElement(root, 'param', name='/backtrack_node/log_split_size_mb',
                          value=str(split_mb), type='int')
            if invalid_config:
                config = output / 'invalid_trajectory.yaml'
                config.write_text('v_max: [invalid value]\n')
                ET.SubElement(root, 'param', name='/backtrack_node/trajectory_config_path',
                              value=str(config), type='str')
            wrapper = output / (name + '.launch')
            ET.ElementTree(root).write(str(wrapper), encoding='utf-8', xml_declaration=True)
            proc = start(['roslaunch', str(wrapper)], name)
            expected = 'failed' if name == 'bad_split' else 'recording' if enabled else 'disabled'
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                ready = any(m.get('state') == expected for m in health)
                if ready and (invalid_config or not enabled or len(controls) >= 5):
                    break
                time.sleep(.05)
            else:
                raise AssertionError('{} launch did not become ready'.format(name))
            time.sleep(1.2)
            received = len(controls)
            stop(proc)
            if name == 'bad_split':
                assert received >= 5, received
                assert not directory.exists()
                assert any('at least 64' in m.get('error', '') for m in health), health
                results[name] = dict(controls=received, creates_log_directory=False,
                                     health='failed', flight_node_continues=True)
                continue
            if not enabled:
                assert received == 0, received
                assert not directory.exists()
                results[name] = dict(controls=0, creates_log_directory=False, health='disabled')
                continue
            import rosbag
            runs = list(directory.glob('run_*'))
            assert len(runs) == 1, runs
            run = runs[0]
            manifest = json.loads((run / 'manifest.json').read_text())
            assert manifest['closed'] and not manifest['interrupted'], manifest
            assert not list(run.glob('*.active'))
            counts = {}
            for path in run.glob('*.bag'):
                with rosbag.Bag(str(path)) as bag:
                    for topic, _, _ in bag.read_messages():
                        counts[topic] = counts.get(topic, 0) + 1
            assert counts.get('/px4_backtrack/log/recorder_status', 0) > 0, counts
            if invalid_config:
                assert received == 0, received
                assert 'invalid value' in manifest['trajectory_config']['yaml'], manifest
                assert manifest['effective_parameters']['trajectory_config_path'] == str(config)
                assert 'Failed to load configuration' in (output / (name + '.log')).read_text()
            else:
                assert counts.get('/px4_backtrack/log/control', 0) >= 5, counts
                assert manifest.get('runtime_configuration', {}).get('build_revision')
            results[name] = dict(control_samples=received, bag_topics=counts,
                                 closed=True, active_bags=0, parameter_snapshot=True,
                                 startup_yaml_snapshot=True)
        result = dict(checks='PASS', scenarios=results,
                      scope='installed launch on isolated ROS master; no FCU connected')
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        rospy.signal_shutdown('finished')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
