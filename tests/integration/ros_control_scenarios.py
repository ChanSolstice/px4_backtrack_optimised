#!/usr/bin/env python3
"""独立 ROS master 上的真实节点联调，不连接 MAVROS/真实飞机。

运行前 source catkin devel/setup.bash；结果和日志写入 --output-dir。
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True)
    parser.add_argument('--config-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    processes = []
    logs = []

    def start(command, name):
        log = (output / f'{name}.log').open('w')
        logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    try:
        start(['roscore', '-p', str(port)], 'master')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1:
                    break
            except OSError:
                time.sleep(0.05)
        else:
            raise RuntimeError('isolated ROS master did not start')

        import rospy
        import yaml
        from mavros_msgs.msg import PositionTarget, RCIn, State
        from mavros_msgs.srv import SetMode, SetModeResponse
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        rospy.init_node('return_contract_test', disable_signals=True)
        params = yaml.safe_load((Path(args.config_dir) / 'backtrack_params.yaml').read_text())
        params.update(record_delay_s=0.1, record_min_points=3, record_max_hz=20.0,
                      ch7_hold_s=0.1, pre_offboard_setpoint_count=0, landing_hover_s=0.2,
                      enable_trigger_ch18=False, enable_trigger_rc_loss=False,
                      enable_trigger_low_battery=False, debug_enable_csv_dump=True,
                      debug_csv_output_dir=str(output / 'csv'), debug_enable_rviz_publish=True,
                      trajectory_config_path=str(Path(args.config_dir) / 'trajectory_config.yaml'))
        for key, value in params.items():
            rospy.set_param('/backtrack_node/' + key, value)

        lock = threading.RLock()
        state = {'armed': False, 'mode': 'POSCTL', 'x': 0.0, 'velocity': 0.0,
                 'y': 0.0, 'yaw': math.pi, 'odom': True, 'ch7': 1500, 'follow': False, 'status': '', 'target': None}
        setpoints, statuses, requests = [], [], []
        blocked = threading.Event()
        service_count = [0]

        def set_mode(req):
            began = time.monotonic()
            with lock:
                service_count[0] += 1
                first = service_count[0] == 1
            if first:
                blocked.set()
                time.sleep(2.0)
            else:
                time.sleep(0.02)
            with lock:
                state['mode'] = req.custom_mode
                requests.append((began, time.monotonic(), req.custom_mode))
            return SetModeResponse(mode_sent=True)

        rospy.Service('/mavros/set_mode', SetMode, set_mode)
        state_pub = rospy.Publisher('/mavros/state', State, queue_size=1)
        odom_pub = rospy.Publisher('/Odometry_px4', Odometry, queue_size=1)
        rc_pub = rospy.Publisher('/mavros/rc/in', RCIn, queue_size=1)

        def on_target(msg):
            with lock:
                state['target'] = msg
                setpoints.append((time.monotonic(), msg))

        def on_status(msg):
            with lock:
                state['status'] = msg.data
                statuses.append((time.monotonic(), msg.data))

        rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, on_target, queue_size=200)
        rospy.Subscriber('/px4_backtrack/state', String, on_status, queue_size=200)

        def publish(_):
            with lock:
                if state['follow'] and state['status'] == 'ReturnActive' and state['target']:
                    delta = state['target'].position.x - state['x']
                    motion = max(-0.002, min(0.002, delta))
                    state['x'] += motion
                    state['velocity'] = motion / 0.02
                else:
                    state['velocity'] = 0.0
                stamp = rospy.Time.now()
                fcu = State(connected=True, armed=state['armed'], mode=state['mode'])
                fcu.header.stamp = stamp
                state_pub.publish(fcu)
                rc = RCIn()
                rc.header.stamp = stamp
                rc.channels = [1500] * 18
                rc.channels[6] = state['ch7']
                rc_pub.publish(rc)
                if state['odom']:
                    odom = Odometry()
                    odom.header.stamp = stamp
                    odom.header.frame_id = 'map'
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose.position.x = state['x']
                    odom.pose.pose.position.y = state['y']
                    odom.pose.pose.position.z = 1.0
                    odom.pose.pose.orientation.z = math.sin(state['yaw']/2)
                    odom.pose.pose.orientation.w = math.cos(state['yaw']/2)
                    odom.twist.twist.linear.x = -state['velocity']  # child frame
                    odom_pub.publish(odom)

        timer = rospy.Timer(rospy.Duration(0.02), publish)
        node = start([args.node, '__name:=backtrack_node'], 'node')

        def wait_status(expected, timeout=8.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                with lock:
                    if state['status'] == expected:
                        return
                if node.poll() is not None:
                    raise AssertionError('node exited; inspect node.log')
                time.sleep(0.02)
            raise AssertionError(f'expected {expected}, got {state["status"]}')

        def press():
            with lock:
                state['ch7'] = 1000
            time.sleep(0.25)
            with lock:
                state['ch7'] = 1500
            time.sleep(0.1)

        def flight_line(length, duration):
            with lock:
                state.update(armed=False, mode='POSCTL', x=0.0, follow=False)
            wait_status('IdleDisarmed')
            time.sleep(0.2)
            with lock:
                state['armed'] = True
            wait_status('Recording')
            began = time.monotonic()
            while time.monotonic() - began < duration:
                with lock:
                    state['x'] = length * min(1.0, (time.monotonic() - began) / duration)
                time.sleep(0.02)
            with lock:
                state['x'] = length
            time.sleep(0.2)

        wait_status('IdleDisarmed')
        flight_line(0.6, 2.0)
        press()
        assert blocked.wait(3.0), 'mode service was not called'
        wait_status('ReturnActive')
        time.sleep(4.0)
        with lock:
            began, ended, _ = requests[0]
            heartbeat = [t for t, _ in setpoints if began <= t <= ended]
            target = state['target']
            assert len(heartbeat) >= 30, len(heartbeat)
            max_gap = max(b-a for a, b in zip(heartbeat, heartbeat[1:]))
            assert max_gap < 0.15, max_gap
            assert abs(target.position.x - state['x']) <= 0.201
            assert abs(target.velocity.x) < 0.03
            state['odom'] = False
        wait_status('Paused')
        with lock:
            assert abs(state['target'].velocity.x) < 1e-9
            state['odom'] = True
        time.sleep(0.3)
        press()
        wait_status('ReturnActive')
        with lock:
            state['mode'] = 'ALTCTL'
        wait_status('ControlYield')
        time.sleep(0.2)  # 清掉 ROS 传输中的已有帧
        with lock:
            count = len(setpoints)
            request_count = len(requests)
        time.sleep(0.5)
        with lock:
            assert len(setpoints) == count
            assert len(requests) == request_count
        press()
        wait_status('ReturnActive')
        with lock:
            state['follow'] = True
        wait_status('LandingHover', 30.0)
        with lock:
            endpoint_error = abs(state['x'])
            assert endpoint_error <= 0.20
        wait_status('AutoLandYield')
        with lock:
            state['armed'] = False
        wait_status('IdleDisarmed')

        # 重新启动同一个真实节点，用合法且严格的走廊配置制造 FIT_FAILED。
        os.killpg(node.pid, signal.SIGTERM)
        node.wait(timeout=5)
        cfg = yaml.safe_load((Path(args.config_dir) / 'trajectory_config.yaml').read_text())
        cfg.update(max_deviation_m=0.001, corner_blend_len_m=0.03, ds_check_m=0.001)
        tight_config = output / 'tight_corridor.yaml'
        tight_config.write_text(yaml.safe_dump(cfg))
        rospy.set_param('/backtrack_node/trajectory_config_path', str(tight_config))
        with lock:
            state.update(armed=False, mode='POSCTL', x=0.0, y=0.0, yaw=-math.pi/2, follow=False, status='')
        node = start([args.node, '__name:=backtrack_node'], 'fallback_node')
        wait_status('IdleDisarmed')
        time.sleep(0.2)
        with lock:
            state['armed'] = True
        wait_status('Recording')
        for axis in ('x', 'y'):
            began_leg = time.monotonic()
            while time.monotonic()-began_leg < 1.5:
                with lock:
                    state[axis] = 0.5*min(1.0,(time.monotonic()-began_leg)/1.5)
                time.sleep(0.02)
            with lock:
                state[axis] = 0.5
        time.sleep(0.2)
        fallback_began = time.monotonic()
        press()
        wait_status('ReturnActive')
        time.sleep(0.2)
        with lock:
            masks = [m.type_mask for t,m in setpoints if t>=fallback_began]
            assert 0x09F8 in masks, masks
            state['mode']='AUTO.LAND'
        wait_status('AutoLandYield')
        result = dict(blocked_rpc_s=ended-began, heartbeat_count=len(heartbeat),
                      max_publish_gap_s=max_gap, slow_flight_speed_m_s=0.10,
                      endpoint_error_m=endpoint_error, fallback_position_yaw_seen=True,
                      stale_odom_pause=True, external_mode_yield=True, checks='PASS')
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False), flush=True)
        timer.shutdown()
        rospy.signal_shutdown('finished')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
