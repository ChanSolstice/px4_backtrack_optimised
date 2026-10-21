#!/usr/bin/env python3
"""在 PX4 SITL（SIH）中确认重构方案依赖的两条飞控行为（系统性分析第 6 节）。

N1：OFFBOARD 中 setpoint 中断超过 COM_OF_LOSS_T，飞控回退到 POSCTL；setpoint 恢复后
    自动回到 OFFBOARD，不需要再请求模式（D22 的前提）。
N2：触地后摇杆回中，或 OFFBOARD 原地保持（setpoint 不带下降速度）时，飞控不判定落地、
    不自动上锁（D23 的前提）。对照：摇杆保持下降时判定落地并自动上锁。

遥控输入用 MANUAL_CONTROL 模拟（COM_RC_IN_MODE=1），摇杆回中对应机载失控帧的输出。
SIH 的估计高度在飞行中会漂移，触地按下降速度消失判定；原地保持时漂移还可能把飞机带离
地面，所以 N2a、N2b 期间飞机是否确实在地面，飞行结束后用 ULog 中的真值确认（--ulog）。
由 scripts/sitl/run_sitl_checks.sh check 调用。退出码：0 全部符合；1 与预期不符；2 起飞等准备
失败；3（--ulog）N2 期间飞机被带离地面，本次 N2 无效，重跑即可。
"""
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, ManualControl, ParamValue, State
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode

LANDED = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF", 4: "LANDING"}
ON_GROUND = 1
CENTER = 500  # MANUAL_CONTROL 油门回中
OBSERVE_S = 5.0  # N2 每段观察时间：落地检测约 1 s，观察太久估计高度漂移会把飞机带离地面


class Vehicle:
    """订阅飞控状态，并以 20 Hz 发布摇杆和（可选的）位置 setpoint。"""

    def __init__(self):
        self.t0 = time.time()
        self.state = State()
        self.landed = 0
        self.position = None
        self.orientation = None
        self.vz = 0.0  # 向上为正 (m/s)
        self.clearance = math.nan  # 离地高度 (m)
        self.setpoint = None  # (x, y, z)；None 时不发布 setpoint
        self.setpoint_q = None  # setpoint 的姿态：保持设定时的航向，避免在地面转向
        self.throttle = CENTER
        self.landed_seen = []
        rospy.Subscriber("/mavros/state", State, self._on_state)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self._on_extended)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self._on_velocity)
        rospy.Subscriber("/mavros/altitude", Altitude, self._on_altitude)
        self._sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self._mc_pub = rospy.Publisher("/mavros/manual_control/send", ManualControl, queue_size=10)
        threading.Thread(target=self._stream, daemon=True).start()

    def now(self):
        return time.time() - self.t0

    def log(self, text):
        print(f"[{self.now():7.2f}] {text}", flush=True)

    def hold(self, x, y, z):
        self.setpoint_q = self.orientation
        self.setpoint = (x, y, z)

    def _on_state(self, msg):
        if msg.mode != self.state.mode or msg.armed != self.state.armed:
            self.log(f"飞控 mode={msg.mode} armed={msg.armed}")
        self.state = msg

    def _on_extended(self, msg):
        if msg.landed_state != self.landed:
            self.log(f"landed_state={LANDED.get(msg.landed_state, msg.landed_state)}")
        self.landed = msg.landed_state
        self.landed_seen.append((self.now(), msg.landed_state))

    def _on_pose(self, msg):
        p = msg.pose.position
        self.position = (p.x, p.y, p.z)
        self.orientation = msg.pose.orientation

    def _on_velocity(self, msg):
        self.vz = msg.twist.linear.z

    def _on_altitude(self, msg):
        self.clearance = msg.bottom_clearance  # 估计器给出的离地高度，空中可用

    def _stream(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            control = ManualControl()
            control.z = self.throttle
            self._mc_pub.publish(control)
            if self.setpoint is not None:
                sp = PoseStamped()
                sp.header.stamp = rospy.Time.now()
                sp.header.frame_id = "map"
                sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = self.setpoint
                sp.pose.orientation = self.setpoint_q
                self._sp_pub.publish(sp)
            rate.sleep()


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
    raise RuntimeError(f"设置参数 {name} 失败")


def get_param(name):
    result = service("/mavros/param/get", ParamGet)(param_id=name)
    return result.value.integer if result.value.integer != 0 else result.value.real


def request_mode(vehicle, mode, timeout_s=10.0):
    set_mode = service("/mavros/set_mode", SetMode)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        set_mode(custom_mode=mode)
        if wait_until(lambda: vehicle.state.mode == mode, 1.5):
            return True
    return False


def observe(vehicle, seconds):
    """观察一段时间：返回期间是否出现 ON_GROUND、是否一直解锁，以及离地高度范围。"""
    start = vehicle.now()
    armed_always = True
    h_min, h_max = math.inf, -math.inf
    while vehicle.now() - start < seconds:
        armed_always &= vehicle.state.armed
        h_min, h_max = min(h_min, vehicle.clearance), max(h_max, vehicle.clearance)
        time.sleep(0.05)
    on_ground = any(t >= start and s == ON_GROUND for t, s in vehicle.landed_seen)
    return on_ground, armed_always, h_min, h_max


def main():
    rospy.init_node("backtrack_sitl_check", anonymous=True)
    v = Vehicle()
    if not wait_until(lambda: v.state.connected, 60):
        print("MAVROS 未连上飞控")
        return 2

    set_param("COM_RC_IN_MODE", 1)  # 只用 MAVLink 摇杆（MANUAL_CONTROL）
    names = ["COM_OF_LOSS_T", "COM_OBL_RC_ACT", "COM_RC_OVERRIDE", "LNDMC_Z_VEL_MAX",
             "LNDMC_TRIG_TIME", "COM_DISARM_LAND", "MPC_Z_VEL_MAX_DN"]
    v.log("参数：" + "，".join(f"{n}={get_param(n)}" for n in names))

    if not wait_until(lambda: v.position is not None, 120):
        print("没有本地位置")
        return 2

    # 起飞：OFFBOARD 定点到当前位置上方 3 m。估计器对准前位置会漂移、对准时会重置，
    # 所以每次尝试解锁前都把起飞点更新为当前位置（估计器正常后才能解锁）
    arming = service("/mavros/cmd/arming", CommandBool)

    def try_arm():
        x, y, z = v.position
        v.hold(x, y, z + 3.0)
        time.sleep(0.5)
        return request_mode(v, "OFFBOARD", 2.0) and arming(value=True).success and \
            wait_until(lambda: v.state.armed, 1.0)

    if not wait_until(try_arm, 120, period_s=1.0):
        print("起飞失败：无法解锁或进入 OFFBOARD")
        return 2
    if not wait_until(lambda: v.clearance > 1.5, 30):
        print(f"起飞失败：没有离地 1.5 m（{v.clearance:.2f} m）")
        return 2
    time.sleep(5.0)
    results = {}

    # ---- N1：本模块断流后飞控回退，setpoint 恢复后自动回到 OFFBOARD ----
    v.log("N1：停止发布 setpoint 3.5 s（COM_OF_LOSS_T 之后回退）")
    hold = v.setpoint
    v.setpoint = None
    t_stop = v.now()
    fell_back = wait_until(lambda: v.state.mode == "POSCTL", 3.4)
    t_fallback = v.now() - t_stop
    _, _, h_min, h_max = observe(v, max(0.0, 3.5 - (v.now() - t_stop)))
    v.setpoint = hold  # 只恢复发布，不再请求模式
    t_resume = v.now()
    returned = wait_until(lambda: v.state.mode == "OFFBOARD", 5.0)
    t_return = v.now() - t_resume
    results["N1"] = fell_back and returned
    n1 = (f"回退到 POSCTL：{'是' if fell_back else '否'}（停发后 {t_fallback:.1f} s 观察到）；"
          f"恢复发布后自动回到 OFFBOARD：{'是' if returned else '否'}（{t_return:.1f} s，期间未请求模式）；"
          f"回退期间离地 {h_min:.2f}～{h_max:.2f} m")
    v.log("N1 " + n1)
    time.sleep(3.0)

    # ---- N2a：POSCTL 下降，触地瞬间摇杆回中 ----
    v.log("N2a：POSCTL 油门最低下降，触地瞬间回中（触地瞬间关遥控器）")
    v.setpoint = None
    request_mode(v, "POSCTL")
    v.throttle = 0
    wait_until(lambda: v.vz < -0.3, 10)
    touched = wait_until(lambda: v.vz > -0.15, 30, period_s=0.01)  # 下降速度消失即触地
    v.throttle = CENTER
    v.log(f"触地（landed_state={LANDED.get(v.landed, v.landed)}），摇杆回中")
    on_ground, armed, _, _ = observe(v, OBSERVE_S)
    results["N2a"] = touched and not on_ground and armed
    n2a = (f"回中后 {OBSERVE_S:.0f} s：{'判定落地' if on_ground else '没有判定落地'}，"
           f"{'一直解锁' if armed else '已上锁'}")
    v.log("N2a " + n2a)

    # ---- N2b：OFFBOARD 原地保持（模块悬停、准备阶段触地后的保持） ----
    n2b = "跳过（N2a 中已上锁）"
    if armed:
        v.log("N2b：OFFBOARD 保持当前位置")
        v.hold(*v.position)
        time.sleep(1.0)
        entered = request_mode(v, "OFFBOARD")
        on_ground, armed, _, _ = observe(v, OBSERVE_S)
        results["N2b"] = entered and not on_ground and armed
        n2b = (f"进入 OFFBOARD：{'是' if entered else '否'}；{OBSERVE_S:.0f} s 内"
               f"{'判定落地' if on_ground else '没有判定落地'}，"
               f"{'一直解锁' if armed else '已上锁'}")
        v.log("N2b " + n2b)
    else:
        results["N2b"] = False

    # ---- 对照：摇杆保持下降，飞控判定落地并自动上锁 ----
    control = "跳过（已上锁）"
    if v.state.armed:
        v.log("对照：POSCTL 油门保持最低")
        v.setpoint = None
        request_mode(v, "POSCTL")
        v.throttle = 0
        t0 = v.now()
        landed = wait_until(lambda: v.landed == ON_GROUND, 15)
        t_landed = v.now() - t0
        disarmed = wait_until(lambda: not v.state.armed, 10)
        t_disarmed = v.now() - t0
        results["control"] = landed and disarmed
        control = (f"{t_landed:.1f} s 后{'判定落地' if landed else '仍未判定落地'}，"
                   f"{t_disarmed:.1f} s 后{'自动上锁' if disarmed else '仍未上锁'}")
        v.log("对照 " + control)
    else:
        results["control"] = False
    v.throttle = CENTER

    print("\n==== 结果（PX4 SITL，SIH 四旋翼）====")
    print(f"N1  {'符合' if results['N1'] else '不符合'}：{n1}")
    print(f"N2a {'符合' if results['N2a'] else '不符合'}：{n2a}")
    print(f"N2b {'符合' if results['N2b'] else '不符合'}：{n2b}")
    print(f"对照 {'符合' if results['control'] else '不符合'}：{control}")
    return 0 if all(results.values()) else 1


def check_ulog(path):
    """飞行结束后读 ULog：飞控侧的 N1 时间，以及 N2a、N2b 期间飞机按真值在地面的时间。"""
    from pyulog import ULog

    u = ULog(path, ["vehicle_status", "offboard_control_mode", "vehicle_local_position_groundtruth"])
    data = {d.name: d.data for d in u.data_list}

    def sec(t_us):
        return (t_us - u.start_timestamp) / 1e6

    # N1：第一次 OFFBOARD(14) -> POSCTL(2) 是断流回退，之后第一次回到 14 是自动恢复
    status = data["vehicle_status"]
    nav, t_nav = status["nav_state"], status["timestamp"]
    k = next(i for i in range(1, len(nav)) if nav[i - 1] == 14 and nav[i] == 2)
    m = next(i for i in range(k, len(nav)) if nav[i] == 14)
    sp = data["offboard_control_mode"]["timestamp"]
    last_sp = max(t for t in sp if t < t_nav[k])
    first_sp = min(t for t in sp if t > t_nav[k])
    print(f"N1  飞控侧：最后一个 setpoint 后 {sec(t_nav[k]) - sec(last_sp):.2f} s 回退 POSCTL；"
          f"恢复发布后 {sec(t_nav[m]) - sec(first_sp):.2f} s 回到 OFFBOARD")

    # N2：首次触地（真值高度在最高点之后第一次低于 1 cm）后，模式依次是 N2a 的 POSCTL、
    # N2b 的 OFFBOARD、对照的 POSCTL。N2a、N2b 的结论只在飞机确实在地面时成立：
    # 各段连续在地面的时间要明显超过落地检测所需的约 1 s，这里要求 3 s
    gt = data["vehicle_local_position_groundtruth"]
    t_gt, h = gt["timestamp"], [-z for z in gt["z"]]
    peak = max(range(len(h)), key=h.__getitem__)
    touch = next(i for i in range(peak, len(h)) if h[i] < 0.01)
    switches = [t_nav[i] for i in range(1, len(nav)) if nav[i] != nav[i - 1] and t_nav[i] > t_gt[touch]]
    if len(switches) < 2:
        print("N2  ULog 中没有找到 N2a、N2b 两段")
        return 1
    ok = True
    for name, t0, t1 in (("N2a", t_gt[touch], switches[0]), ("N2b", switches[0], switches[1])):
        longest = run = h_max = 0.0
        for i in range(1, len(h)):
            if not t0 <= t_gt[i] <= t1:
                continue
            h_max = max(h_max, h[i])
            run = run + (t_gt[i] - t_gt[i - 1]) / 1e6 if h[i] < 0.02 else 0.0
            longest = max(longest, run)
        ok &= longest >= 3.0
        print(f"{name} 真值：{sec(t0):.1f}～{sec(t1):.1f} s 中连续在地面 {longest:.1f} s，最高 {h_max:.2f} m"
              + ("" if longest >= 3.0 else "；在地面不足 3 s，本次无效（估计高度漂移把飞机带离地面），请重跑"))
    return 0 if ok else 3


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--ulog":
        sys.exit(check_ulog(sys.argv[2]))
    sys.exit(main())
