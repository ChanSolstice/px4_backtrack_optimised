#!/usr/bin/env python3
"""在 PX4 SITL（SIH）中运行返航节点，回放方案第 11 节中涉及飞控交互的场景（方案第 14.3 节，R-86）。

脚本扮演三个角色：
- 接收机：50 Hz 经 MAVROS 遥控覆盖（/mavros/rc/override）向 PX4 发 18 个通道。CH7 按下 1100、松开 1900，
  CH18 正常 1400；失控时 CH18 = 800，CH7 保持失控瞬间的值，摇杆回中；接收机断线（F8）时停发。
  PX4 按机载的 COM_RC_IN_MODE=0 把它当作遥控，再回传 RC_CHANNELS，节点读真实的 /mavros/rc/in（R-91）；
- 飞手：用遥控的摇杆通道在 POSCTL 下起飞、飞出航线；拨模式开关、地面站切模式都用 /mavros/set_mode 模拟；
- 电池：1 Hz 向 /sitl/battery 发电量，backtrack_sitl.launch 把节点的 /mavros/battery 重映射到这里。
飞控的遥控和失控保护参数按机载值设置（R-90），里程计用 /mavros/local_position/odom。判定看节点的状态话题、/mavros/state 和调试日志 events.jsonl；每个场景
还检查 JSON 状态话题与 /px4_backtrack/state 一致、逐拍记录 trace.csv 每行的列数与表头一致。

用法：sitl_scenarios.py <场景编号> <调试日志目录>，由 scripts/sitl/run_sitl_checks.sh scenarios 调用。
退出码：0 检查全部通过；1 有检查失败；2 起飞等准备失败。
"""
import glob
import json
import math
import os
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, ParamValue, PositionTarget, State
from nav_msgs.msg import Path
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

CH7_PRESSED, CH7_RELEASED = 1100, 1900
CH18_OK, CH18_LOST = 1400, 800
PWM_CENTER = 1500
THROTTLE_CENTER = 500  # 油门杆量范围 [0, 1000]，回中时 POSCTL 下保持高度
# 与机载一致的遥控和失控保护参数（方案 R-90）。SITL 默认没有映射摇杆通道；RC_CHAN_CNT 为 0 时 PX4 认为遥控未校准
MACHINE_PARAMS = [
    ("COM_RC_IN_MODE", 0), ("RC_CHAN_CNT", 18), ("RC_MAP_ROLL", 1), ("RC_MAP_PITCH", 2), ("RC_MAP_THROTTLE", 3),
    ("RC_MAP_YAW", 4),
    ("RC_FAILS_THR", 0), ("COM_RC_OVERRIDE", 1), ("COM_RC_STICK_OV", 30.0), ("COM_RC_LOSS_T", 5.0),
    ("NAV_RCL_ACT", 1), ("COM_RCL_EXCEPT", 5), ("COM_FAIL_ACT_T", 5.0), ("COM_OBL_RC_ACT", 0),
    ("COM_OF_LOSS_T", 1.0), ("NAV_DLL_ACT", 0), ("COM_LOW_BAT_ACT", 0),
]
# SIH 中 OFFBOARD、AUTO.LAND 的位置保持误差约 0.3 m（平均）～0.7 m（最大），与本模块的 setpoint 格式无关
# （只发位置时也一样），POSCTL 悬停在 0.1 m 以内。所以这两个距离只能放宽到这里
LANDED_NEAR_M = 1.5    # 在返航终点附近降落、经过退出位置的水平距离上限
S15_MINUTES = float(os.environ.get("S15_MINUTES", "5"))


class PrepareError(Exception):
    """起飞等准备步骤失败，与被测行为无关"""


def wait_until(predicate, timeout_s, period_s=0.05):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


def service(name, srv_type):
    rospy.wait_for_service(name, timeout=60)
    return rospy.ServiceProxy(name, srv_type)


def set_param(name, value):
    param_set = service("/mavros/param/set", ParamSet)
    v = ParamValue(integer=value) if isinstance(value, int) else ParamValue(real=value)
    for _ in range(90):  # MAVROS 拉完参数表之前设置会失败
        try:
            if param_set(param_id=name, value=v).success:
                return
        except rospy.ServiceException:
            pass
        time.sleep(1.0)
    raise PrepareError(f"设置参数 {name} 失败")


class World:
    """飞手、接收机和电池，以及对飞控、返航节点的观察。"""

    def __init__(self):
        self.t0 = time.time()
        self.fcu = State()
        self.position = None  # ENU (x, y, z)
        self.module_state = ""
        self.states = []       # /px4_backtrack/state 依次出现的状态
        self.status_states = []  # /px4_backtrack/status 中依次出现的 state
        self.path = []        # 节点发布的返航轨迹（规划完成时）
        self.max_offset = 0.0  # 跟随中离返航轨迹的最大距离
        self.setpoint = None   # 节点最近一次发布的 setpoint（位置、速度）
        self.track = []        # (t, 节点状态, 位置, setpoint)，写到日志目录供分析
        self.stick = [0, 0, THROTTLE_CENTER, 0]  # x 向前、y 向右、z 油门、r 偏航
        self.link_ok = True
        self.rc_frames = True  # 接收机断线（F8）时为假，遥控帧完全中断
        self.ch7_pressed = False
        self.battery = 0.9
        self.first_failsafe_t = None  # 第一帧失控帧的时间（ROS 时间）
        self._held_ch7 = False
        rospy.Subscriber("/mavros/state", State, self._on_state)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose)
        rospy.Subscriber("/px4_backtrack/state", String, self._on_module_state)
        rospy.Subscriber("/px4_backtrack/status", String, self._on_status)
        rospy.Subscriber("/px4_backtrack/debug/trajectory", Path, self._on_path)
        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, self._on_setpoint)
        self._rc_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self._battery_pub = rospy.Publisher("/sitl/battery", BatteryState, queue_size=10)
        self._set_mode = service("/mavros/set_mode", SetMode)
        threading.Thread(target=self._stream, daemon=True).start()

    def log(self, text):
        print(f"[{time.time() - self.t0:7.2f}] {text}", flush=True)

    # ---- 观察 ----
    def _on_state(self, msg):
        if msg.mode != self.fcu.mode or msg.armed != self.fcu.armed:
            self.log(f"飞控 mode={msg.mode} armed={msg.armed}")
        self.fcu = msg

    def _on_pose(self, msg):
        p = msg.pose.position
        self.position = (p.x, p.y, p.z)
        offset = math.nan
        if self.path and self.module_state in ("ReturnActive", "ReturnRejoin"):
            offset = min(math.dist(self.position, q) for q in self.path)
            self.max_offset = max(self.max_offset, offset)
        self.track.append((rospy.get_time(), self.module_state, self.position, self.setpoint, offset))

    def _on_setpoint(self, msg):
        p, v = msg.position, msg.velocity
        self.setpoint = (p.x, p.y, p.z, v.x, v.y, v.z)

    def save_track(self, log_dir):
        with open(os.path.join(log_dir, "track.csv"), "w", encoding="utf-8") as f:
            f.write("t,state,x,y,z,sp_x,sp_y,sp_z,sp_vx,sp_vy,sp_vz,offset\n")
            for t, state, pos, sp, offset in self.track:
                sp = sp or (math.nan,) * 6
                f.write(f"{t:.3f},{state}," + ",".join(f"{v:.3f}" for v in (*pos, *sp, offset)) + "\n")
        with open(os.path.join(log_dir, "path.csv"), "w", encoding="utf-8") as f:
            f.write("x,y,z\n")
            for q in self.path:
                f.write(",".join(f"{v:.3f}" for v in q) + "\n")

    def _on_path(self, msg):
        self.path = [(q.pose.position.x, q.pose.position.y, q.pose.position.z) for q in msg.poses]

    def _on_module_state(self, msg):
        if msg.data != self.module_state:
            self.log(f"节点 {msg.data}")
            self.states.append(msg.data)
        self.module_state = msg.data

    def _on_status(self, msg):
        state = json.loads(msg.data)["state"]
        if not self.status_states or self.status_states[-1] != state:
            self.status_states.append(state)

    def wait_state(self, state, timeout_s):
        return wait_until(lambda: self.module_state == state, timeout_s)

    def horizontal_distance(self, point):
        return math.hypot(self.position[0] - point[0], self.position[1] - point[1])

    # ---- 飞手、接收机、电池 ----
    def set_mode(self, mode):
        """飞手拨模式开关或地面站切模式"""
        self.log(f"切换模式 {mode}")
        self._set_mode(custom_mode=mode)

    def fly(self, forward, right, seconds):
        """POSCTL 下推杆 seconds 秒后回中；杆量范围 [-1000, 1000]"""
        self.stick[0], self.stick[1] = forward, right
        time.sleep(seconds)
        self.stick[0] = self.stick[1] = 0

    def press_ch7(self, seconds):
        self.ch7_pressed = True
        time.sleep(seconds)
        self.ch7_pressed = False

    def set_link(self, ok):
        self.log("链路恢复" if ok else "链路丢失（接收机失控）")
        self.link_ok = ok

    def stop_rc_frames(self):
        """接收机断线（F8）：遥控帧完全中断"""
        self.log("遥控帧中断（接收机断线）")
        self.rc_frames = False

    def _stream(self):
        """50 Hz 发遥控帧，1 Hz 发电量。失控时接收机输出回中的摇杆，CH7 保持失控瞬间的值。"""
        rate = rospy.Rate(50)
        k = 0
        while not rospy.is_shutdown():
            link_ok = self.link_ok
            if link_ok:
                self._held_ch7 = self.ch7_pressed
            elif self.first_failsafe_t is None:
                self.first_failsafe_t = rospy.get_time()
            x, y, z, r = self.stick if link_ok else [0, 0, THROTTLE_CENTER, 0]
            channels = [PWM_CENTER] * 18
            channels[0], channels[1], channels[3] = PWM_CENTER + y // 2, PWM_CENTER + x // 2, PWM_CENTER + r // 2
            channels[2] = 1000 + z
            channels[6] = CH7_PRESSED if (self.ch7_pressed if link_ok else self._held_ch7) else CH7_RELEASED
            channels[17] = CH18_OK if link_ok else CH18_LOST
            try:
                if self.rc_frames:
                    self._rc_pub.publish(OverrideRCIn(channels=channels))
                if k % 50 == 0:
                    battery = BatteryState()
                    battery.header.stamp = rospy.Time.now()
                    battery.percentage = self.battery
                    self._battery_pub.publish(battery)
                rate.sleep()
            except rospy.ROSException:  # 节点关闭
                return
            k += 1


class Events:
    """返航节点调试日志 events.jsonl 中的事件"""

    def __init__(self, log_dir):
        self.items = []
        for path in glob.glob(os.path.join(log_dir, "run_*", "events.jsonl")):
            with open(path, encoding="utf-8") as f:
                for line in f:
                    event = json.loads(line)
                    if event.get("type") != "config":
                        self.items.append(event)
        self.items.sort(key=lambda e: e["t"])

    def find(self, type_, name=None, to=None, after=0.0):
        return [e for e in self.items if e["type"] == type_ and e["t"] >= after
                and (name is None or e.get("name") == name) and (to is None or e.get("to") == to)]

    def requests(self, mode, after=0.0):
        """发出的模式请求"""
        return self.find("mode_request", "sent", mode, after)

    def alert_on(self, alert):
        return bool(self.find("alert", alert, "on"))


def ulog_data(topic):
    """本次飞行 PX4 ULog 中一个话题的数据（上锁后 ULog 才写完）"""
    from pyulog import ULog

    path = max(glob.glob("/px4/build/px4_sitl_default/rootfs/log/*/*.ulg"), key=os.path.getmtime)
    return ULog(path, [topic]).data_list[0].data


def ulog_modes():
    """本次飞行 PX4 ULog 中，解锁期间依次出现的飞行模式"""
    names = {1: "ALTCTL", 2: "POSCTL", 4: "AUTO.LOITER", 5: "AUTO.RTL", 12: "DESCEND", 14: "OFFBOARD",
             18: "AUTO.LAND"}
    status = ulog_data("vehicle_status")
    modes = []
    for nav, arming in zip(status["nav_state"], status["arming_state"]):
        if arming != 2:  # ARMING_STATE_ARMED
            continue
        name = names.get(int(nav), str(int(nav)))
        if not modes or modes[-1] != name:
            modes.append(name)
    return modes


def check_reports(w, c, log_dir):
    """每个场景都检查：状态话题与 /px4_backtrack/state 一致；trace.csv 每行的列数与表头一致"""
    time.sleep(0.5)  # 两个话题的最后一条都已收到
    c.add("状态话题的 state 与 /px4_backtrack/state 一致", w.status_states == w.states,
          f"状态 {len(w.states)} 个：{' → '.join(w.status_states[-4:])}")
    rows = bad = 0
    for path in glob.glob(os.path.join(log_dir, "run_*", "trace.csv")):
        with open(path, encoding="utf-8") as f:
            columns = f.readline().count(",")
            for line in f:
                rows += 1
                bad += line.count(",") != columns
    c.add("逐拍记录每行的列数与表头一致", rows > 0 and bad == 0, f"{rows} 行，列数不符 {bad} 行")


class Checks:
    def __init__(self):
        self.items = []  # (名称, 是否通过；None 为只记录, 说明)

    def add(self, name, ok, detail):
        self.items.append((name, bool(ok), detail))

    def note(self, name, detail):
        self.items.append((name, None, detail))


# ============================================================================
# 共用步骤
# ============================================================================

def take_off(w, airborne=True):
    """POSCTL 下解锁（油门最低），起飞到约 3 m，等到节点开始录制；返回录制起点"""
    for name, value in MACHINE_PARAMS:
        set_param(name, value)
    set_param("COM_DISARM_PRFLT", -1.0)   # 在地面解锁后不自动上锁（S13）
    if not wait_until(lambda: w.position is not None, 120):
        raise PrepareError("没有本地位置")
    arming = service("/mavros/cmd/arming", CommandBool)
    w.stick[2] = 0

    def try_arm():
        if w.fcu.mode != "POSCTL":
            w.set_mode("POSCTL")
            wait_until(lambda: w.fcu.mode == "POSCTL", 1.5)
        return w.fcu.mode == "POSCTL" and arming(value=True).success and \
            wait_until(lambda: w.fcu.armed, 1.0)

    # 估计器对准之前不能解锁
    if not wait_until(try_arm, 120, period_s=1.0):
        raise PrepareError("POSCTL 下无法解锁")
    if airborne:
        w.stick[2] = 800
        if not wait_until(lambda: w.position[2] > 2.5, 20):
            raise PrepareError(f"起飞失败：高度 {w.position[2]:.2f} m")
        w.stick[2] = THROTTLE_CENTER
    if not w.wait_state("Recording", 25):
        raise PrepareError(f"节点没有开始录制（{w.module_state}）")
    time.sleep(1.0)
    return w.position


def fly_outbound(w):
    """向前约 10 m，再向右约 8 m，悬停"""
    w.fly(300, 0, 8.0)
    w.fly(0, 300, 6.0)
    time.sleep(2.0)


def start_low_battery_return(w):
    w.log("低电量")
    w.battery = 0.10
    if not w.wait_state("ReturnActive", 20):
        raise PrepareError(f"低电量后没有开始返航（{w.module_state}）")


def check_landed_at_start(w, c, timeout_s=150):
    """在返航终点（录制起点）附近降落并上锁；跟随中离返航轨迹不超过安全距离"""
    landed = wait_until(lambda: not w.fcu.armed, timeout_s, period_s=0.2)
    distance = w.horizontal_distance(w.path[-1]) if w.path else math.inf
    c.add("返航完成并在录制起点上锁", landed and distance < LANDED_NEAR_M,
          f"{'已上锁' if landed else '未上锁'}，距返航终点 {distance:.2f} m，节点 {w.module_state}")
    c.note("跟随中离返航轨迹的最大距离", f"{w.max_offset:.2f} m（含 SIH 的位置保持误差）")


def check_never_exited(c, ev):
    exited = ev.find("state", to="Exited")
    c.add("任务不被非预期结束", not exited, "进入了 Exited" if exited else "未退出")


# ============================================================================
# 场景（方案第 11 节）
# ============================================================================

def s1(w, c, log_dir):
    """S1：低电量返航中失控，继续返航；交接不经过 POSCTL"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    time.sleep(3.0)
    w.set_link(False)
    lost_t = rospy.get_time()
    check_landed_at_start(w, c)
    ev = Events(log_dir)
    check_never_exited(c, ev)
    posctl = ev.requests("POSCTL", after=lost_t)
    c.add("链路丢失后不请求 POSCTL", not posctl, f"请求 POSCTL {len(posctl)} 次")


def s4(w, c, log_dir):
    """S4：录制中单独失控：1.5 s 内申请 OFFBOARD，悬停 15 s 后返航"""
    take_off(w)
    fly_outbound(w)
    w.set_link(False)
    wait_until(lambda: w.first_failsafe_t is not None, 1.0)
    check_landed_at_start(w, c, timeout_s=180)
    ev = Events(log_dir)
    offboard = ev.requests("OFFBOARD")
    delay = offboard[0]["t"] - w.first_failsafe_t if offboard else math.inf
    c.add("第一帧失控帧后 1.5 s 内申请 OFFBOARD", delay <= 1.5, f"{delay:.2f} s")
    hover = ev.find("state", to="Ch18HoverWait")
    prepare = ev.find("state", to="ReturnPreOffboard")
    hover_s = prepare[0]["t"] - hover[0]["t"] if hover and prepare else 0.0
    c.add("悬停 15 s 后返航", 14.9 <= hover_s <= 15.5, f"悬停 {hover_s:.2f} s")


def s6(w, c, log_dir):
    """S6：返航中链路正常时短按 CH7：退出，截断轨迹，请求 POSCTL"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    time.sleep(3.0)
    w.press_ch7(0.5)
    exit_t = rospy.get_time()
    exited = w.wait_state("Exited", 2.0)
    posctl = wait_until(lambda: w.fcu.mode == "POSCTL", 3.0)
    time.sleep(3.0)
    ev = Events(log_dir)
    c.add("短按后进入 Exited", exited, w.module_state)
    c.add("退出后飞控进入 POSCTL", posctl, w.fcu.mode)
    c.add("截断录制", bool(ev.find("mission", "trail_cut")),
          (ev.find("mission", "trail_cut") or [{}])[0].get("detail", "没有截断事件"))
    offboard = ev.requests("OFFBOARD", after=exit_t)
    c.add("退出后不再申请 OFFBOARD", not offboard, f"申请 {len(offboard)} 次")


def s7(w, c, log_dir):
    """S7：返航中链路正常时飞手拨模式开关：约 1 s 内退出"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    time.sleep(3.0)
    switch_t = rospy.get_time()
    w.set_mode("POSCTL")
    exited = w.wait_state("Exited", 2.5)
    exit_s = rospy.get_time() - switch_t
    time.sleep(3.0)
    ev = Events(log_dir)
    c.add("拨开关后进入 Exited", exited, f"{exit_s:.2f} s，{w.module_state}")
    offboard = ev.requests("OFFBOARD", after=switch_t)
    c.add("退出后不再申请 OFFBOARD", not offboard and w.fcu.mode == "POSCTL",
          f"申请 {len(offboard)} 次，飞控 {w.fcu.mode}")


def s11(w, c, log_dir):
    """S11：退出后失控：新任务先沿退出后的手动段回到退出位置，再沿截断后的轨迹回到起点"""
    take_off(w)
    fly_outbound(w)
    w.log("长按 CH7")
    w.press_ch7(2.5)
    if not w.wait_state("ReturnActive", 10):
        raise PrepareError(f"长按后没有开始返航（{w.module_state}）")
    time.sleep(4.0)
    w.press_ch7(0.5)
    if not w.wait_state("Exited", 2.0):
        raise PrepareError(f"短按后没有退出（{w.module_state}）")
    wait_until(lambda: w.fcu.mode == "POSCTL", 3.0)
    exit_pos = w.position
    w.fly(0, -300, 5.0)  # 手动向左飞约 6 m
    time.sleep(2.0)
    w.set_link(False)
    lost_t = rospy.get_time()
    closest = [math.inf]

    def landed():
        closest[0] = min(closest[0], w.horizontal_distance(exit_pos))
        return not w.fcu.armed

    wait_until(landed, 200, period_s=0.1)
    ev = Events(log_dir)
    second = ev.find("mission", "started", after=lost_t)
    c.add("退出后失控开始新任务", bool(second), second[0]["detail"] if second else "没有新任务")
    c.add("新任务经过退出位置", closest[0] < LANDED_NEAR_M, f"离退出位置最近 {closest[0]:.2f} m")
    check_landed_at_start(w, c, timeout_s=0)


def s12(w, c, log_dir):
    """S12：链路丢失时被切出 OFFBOARD：重新申请，回到 OFFBOARD 后继续返航"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    time.sleep(3.0)
    w.set_link(False)
    time.sleep(2.0)  # 等链路丢失确认
    switch_t = rospy.get_time()
    w.set_mode("POSCTL")  # 地面站切出
    wait_until(lambda: w.fcu.mode == "POSCTL", 2.0)
    regained = wait_until(lambda: w.fcu.mode == "OFFBOARD", 6.0)
    regain_s = rospy.get_time() - switch_t
    check_landed_at_start(w, c)
    ev = Events(log_dir)
    c.add("被切出后重新申请并回到 OFFBOARD", regained and ev.find("control", to="reclaiming"),
          f"{regain_s:.1f} s 后回到 OFFBOARD" if regained else "没有回到 OFFBOARD")
    check_never_exited(c, ev)


def s13(w, c, log_dir):
    """S13：在地面解锁、未起飞时失控或低电量：不启动任务"""
    take_off(w, airborne=False)
    w.set_link(False)
    time.sleep(5.0)
    w.set_link(True)
    time.sleep(3.0)
    w.battery = 0.10
    time.sleep(6.0)
    ev = Events(log_dir)
    states = [e["to"] for e in ev.find("state")]
    c.add("在地面不启动任务", "Recording" in states and states[-1] == "Recording" and w.module_state == "Recording",
          "状态 " + " → ".join(states))
    c.add("不请求模式、不起飞", not ev.find("mode_request", "sent") and w.position[2] < 0.5 and w.fcu.armed,
          f"模式请求 {len(ev.find('mode_request', 'sent'))} 次，高度 {w.position[2]:.2f} m")
    c.add("告警 TRIGGER_IGNORED_LANDED", ev.alert_on("TRIGGER_IGNORED_LANDED"), "")


def s15(w, c, log_dir):
    """S15：长录制后失控：悬停期间后台规划，控制周期照常发布，OFFBOARD 不回退"""
    take_off(w)
    w.log(f"往返飞行 {S15_MINUTES:.0f} 分钟")
    deadline = time.time() + S15_MINUTES * 60.0
    leg = 0
    while time.time() < deadline:  # 前后往返，横向来回错开，保持在约 15 m 范围内
        w.fly(300 if leg % 2 == 0 else -300, 100 if leg % 4 < 2 else -100, 8.0)
        leg += 1
    time.sleep(2.0)
    w.set_link(False)
    if not w.wait_state("ReturnActive", 60):
        raise PrepareError(f"没有开始返航（{w.module_state}）")
    time.sleep(10.0)
    ev = Events(log_dir)
    fallbacks = [e for e in ev.find("fcu") if e.get("from") == "OFFBOARD" and e.get("to") == "POSCTL"]
    gaps = ev.find("stream")
    planned = ev.find("planning", "completed")
    c.add("后台规划完成", bool(planned), planned[0]["detail"] if planned else "没有完成")
    c.add("OFFBOARD 不因断流回退", not fallbacks and not gaps and w.fcu.mode == "OFFBOARD",
          f"回退 {len(fallbacks)} 次，断流 {len(gaps)} 次，飞控 {w.fcu.mode}")


def s18(w, c, log_dir):
    """S18：降落交接中飞手拨开关：模块看到飞手的模式后退出，不再请求 AUTO.LAND"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    if not w.wait_state("LandingTransition", 120):
        raise PrepareError(f"没有进入降落交接（{w.module_state}）")
    w.set_mode("ALTCTL")
    # 飞控状态 1 Hz：模块看到飞手的模式之前，按旧状态发出的 AUTO.LAND 可能先生效，飞手再拨一次
    if wait_until(lambda: w.fcu.mode == "AUTO.LAND", 3.0):
        w.set_mode("ALTCTL")
    time.sleep(6.0)
    ev = Events(log_dir)
    seen = [e for e in ev.find("fcu") if e.get("to") == "ALTCTL"]
    seen_t = seen[-1]["t"] if seen else math.inf
    autoland = ev.requests("AUTO.LAND", after=seen_t)
    c.add("看到飞手的模式后不再请求 AUTO.LAND", seen and not autoland and w.fcu.mode == "ALTCTL",
          f"之后请求 {len(autoland)} 次，飞控 {w.fcu.mode}，节点 {w.module_state}")
    c.add("交接中看到飞手的模式即退出", w.module_state == "Exited" or not ev.find("state", to="AutoLandYield"),
          w.module_state)


def s21(w, c, log_dir):
    """S21：链路丢失期间，飞控降落中模式被切走：每 1 s 重新请求 AUTO.LAND"""
    take_off(w)
    fly_outbound(w)
    w.set_link(False)
    if not w.wait_state("AutoLandYield", 180):
        raise PrepareError(f"没有进入飞控降落（{w.module_state}）")
    wait_until(lambda: w.position[2] < 2.0, 10)
    switch_t = rospy.get_time()
    w.set_mode("POSCTL")
    switched = wait_until(lambda: w.fcu.mode == "POSCTL", 2.0)
    back = switched and wait_until(lambda: w.fcu.mode == "AUTO.LAND", 5.0)
    landed = wait_until(lambda: not w.fcu.armed, 30, period_s=0.2)
    ev = Events(log_dir)
    again = ev.requests("AUTO.LAND", after=switch_t)
    c.add("模式被切走后重新请求 AUTO.LAND", back and again,
          f"重新请求 {len(again)} 次，飞控 {w.fcu.mode}")
    c.add("告警 AUTOLAND_OVERRIDDEN", ev.alert_on("AUTOLAND_OVERRIDDEN"), "")
    c.add("降落并上锁", landed, "已上锁" if landed else "未上锁")


def s26(w, c, log_dir):
    """S26：返航中本模块卡顿 1.3 s，飞控回退 POSCTL：不退出，飞控自动回到 OFFBOARD"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    time.sleep(3.0)
    w.log("节点卡顿 1.3 s")
    subprocess.run(["pkill", "-STOP", "-x", "backtrack_node"], check=True)
    time.sleep(1.3)
    subprocess.run(["pkill", "-CONT", "-x", "backtrack_node"], check=True)
    check_landed_at_start(w, c)
    ev = Events(log_dir)
    check_never_exited(c, ev)
    # 回退只有零点几秒，1 Hz 的飞控状态多半看不到，按 PX4 的 ULog 判定（上锁后等日志写完）
    time.sleep(1.0)
    modes = ulog_modes()
    returned = any(modes[i:i + 3] == ["OFFBOARD", "POSCTL", "OFFBOARD"] for i in range(len(modes)))
    c.add("飞控回退 POSCTL 后自动回到 OFFBOARD", returned, "ULog 中的模式：" + " → ".join(modes))
    c.add("没有当作外部切出重新申请", not ev.find("control", to="reclaiming"),
          "出现重新申请" if ev.find("control", to="reclaiming") else "没有重新申请")


def s27(w, c, log_dir):
    """S27：返航开始、OFFBOARD 确认前飞手拨开关到 ALTCTL：退出，不再申请 OFFBOARD"""
    take_off(w)
    fly_outbound(w)
    w.battery = 0.10
    if not w.wait_state("ReturnPreOffboard", 10):
        raise PrepareError(f"低电量后没有开始任务（{w.module_state}）")
    w.set_mode("ALTCTL")
    exited = w.wait_state("Exited", 3.0)
    time.sleep(3.0)
    ev = Events(log_dir)
    offboard = ev.requests("OFFBOARD")
    c.add("确认前飞手接管：进入 Exited", exited, w.module_state)
    c.add("没有申请 OFFBOARD", not offboard and w.fcu.mode == "ALTCTL",
          f"申请 {len(offboard)} 次，飞控 {w.fcu.mode}")


def check_rc_frame_loss(w, c, log_dir, stop_t):
    """F8 的共同检查：本模块判定链路丢失、不退出；飞控判定了 RC 丢失，在 OFFBOARD 中不响应；
    交接后进入的 Hold 被本模块重新请求 AUTO.LAND 接管，不来回切换；在返航终点附近降落"""
    check_landed_at_start(w, c, timeout_s=200)
    ev = Events(log_dir)
    lost = ev.find("alert", "LINK_LOST", "on", after=stop_t)
    delay = lost[0]["t"] - stop_t if lost else math.inf
    c.add("帧中断后判定链路丢失", delay < 1.0, f"{delay:.2f} s")
    exited = ev.find("state", to="Exited")
    c.add("任务不被非预期结束", not exited, "退出" if exited else "未退出")
    autoland = ev.requests("AUTO.LAND", after=stop_t)
    land_t = autoland[0]["t"] if autoland else math.inf
    left = [e for e in ev.find("fcu", after=stop_t) if e["t"] < land_t and e.get("from") == "OFFBOARD"]
    c.add("返航中飞控不响应 RC 丢失", not left, f"请求 AUTO.LAND 之前离开 OFFBOARD {len(left)} 次")
    time.sleep(2.0)  # ULog 在上锁后写完
    c.add("飞控判定了 RC 丢失", any(ulog_data("failsafe_flags")["manual_control_signal_lost"]),
          "failsafe_flags.manual_control_signal_lost")
    modes = ulog_modes()
    tail = modes[len(modes) - modes[::-1].index("OFFBOARD"):]
    c.add("交接后的 Hold 被接管，不来回切换", "AUTO.LAND" in tail and tail.count("AUTO.LOITER") <= 1,
          "ULog 中的模式：" + " → ".join(tail))
    return modes


def f8a(w, c, log_dir):
    """F8a：低电量返航中遥控帧完全中断（接收机断线）：继续返航，交接后的 Hold 被接管"""
    take_off(w)
    fly_outbound(w)
    start_low_battery_return(w)
    time.sleep(3.0)
    stop_t = rospy.get_time()
    w.stop_rc_frames()
    check_rc_frame_loss(w, c, log_dir, stop_t)


def f8b(w, c, log_dir):
    """F8b：录制中遥控帧完全中断：在飞控判定 RC 丢失（5 s）之前取得 OFFBOARD，悬停后返航降落"""
    take_off(w)
    fly_outbound(w)
    stop_t = rospy.get_time()
    w.stop_rc_frames()
    modes = check_rc_frame_loss(w, c, log_dir, stop_t)
    first = modes.index("OFFBOARD")
    c.add("飞控判定 RC 丢失之前取得 OFFBOARD", modes[first - 1] == "POSCTL",
          "ULog 中的模式：" + " → ".join(modes[:first + 1]))


SCENARIOS = {"S1": s1, "S4": s4, "S6": s6, "S7": s7, "S11": s11, "S12": s12, "S13": s13,
             "S15": s15, "S18": s18, "S21": s21, "S26": s26, "S27": s27, "F8a": f8a, "F8b": f8b}


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in SCENARIOS:
        print(f"用法：{sys.argv[0]} <{'|'.join(SCENARIOS)}> <调试日志目录>")
        return 2
    name, log_dir = sys.argv[1], sys.argv[2]
    rospy.init_node("backtrack_sitl_scenario", anonymous=True)
    w = World()
    if not wait_until(lambda: w.fcu.connected, 60):
        print("MAVROS 未连上飞控")
        return 2
    c = Checks()
    try:
        SCENARIOS[name](w, c, log_dir)
        check_reports(w, c, log_dir)
    except PrepareError as e:
        print(f"\n## {name}：准备失败：{e}")
        return 2
    finally:
        w.save_track(log_dir)
    print(f"\n## {name} {SCENARIOS[name].__doc__.split('：', 1)[1]}\n")
    for check, ok, detail in c.items:
        print(f"- [{'记录' if ok is None else '通过' if ok else '失败'}] {check}：{detail}")
    return 0 if all(ok is not False for _, ok, _ in c.items) else 1


if __name__ == "__main__":
    sys.exit(main())
