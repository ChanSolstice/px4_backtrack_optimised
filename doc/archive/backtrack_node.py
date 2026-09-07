#!/usr/bin/env python3
import rospy
import math
import threading
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool

class BacktrackNode:
    """
    BacktrackNode 类

    该节点能够记录无人机的航点，并在触发时按逆序返航。
    返航模式可通过以下方式触发：
    - 通道7按钮
    - RC遥控器信号丢失
    - 通道18低于900

    通道7按钮逻辑：
    - 按住2秒：启动返航
    - 任意时刻按下：停止返航
    
    优先级：当飞机触发一键降落（AUTO.LAND）时，优先一键降落，取消所有返航逻辑。

    断点续航：取消返航后再次触发时，从当前位置继续返航剩余航点
    """
    def __init__(self):
        rospy.init_node('backtrack_node', anonymous=True)

        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.status_rc_loss_pub = rospy.Publisher('/status_rc_loss', Bool, queue_size=10)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.current_state = State()
        self.current_pose = None
        self.was_armed = True
        
        self.waypoints = []
        self.recording = False
        self.return_triggered = False
        self.is_waypoints_reversed = False  # Flag: whether waypoints have been reversed

        # Channel 7 button timing variables
        self.channel7_pressed = False
        self.channel7_press_start_time = 0
        self.channel7_on_triggered = False  # Whether 3s hold has triggered start
        self.channel7_off_triggered = False  # Whether stop has been triggered

        # Channel18 wait variables
        self.is_channel18_triggered = False  # Flag: triggered by channel18
        self.hover_wait_counter = 0          # Wait counter
        self.hover_wait_complete = False     # Whether wait is complete
        self.hover_target_pose = None        # Hover target pose (fixed)

        # Landing wait variables
        self.landing_hover_counter = 0       # Pre-landing hover counter
        self.landing_hover_pose = None       # Pre-landing hover target pose

        self.rc_loss_detected = False
        self.channel18_failsafe_detected = False
        self.last_rc_time = time.time()
        self.rc_loss_start_time = 0
        self.rc_loss_trigger_thread = None
        self.channel18_failsafe_thread = None
        self.pre_offboard_counter = 0       # Pre-offboard pose publish counter
        
        self._just_disarmed = False

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.pose_cb)
        self.rc_sub = rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_cb)
      
        self.sp_thread = threading.Thread(target=self.publish_setpoints)
        self.sp_thread.daemon = True
        self.sp_thread.start()

        self.rc_check_thread = threading.Thread(target=self.check_rc_loss)
        self.rc_check_thread.daemon = True
        self.rc_check_thread.start()
        
        rospy.loginfo("Backtrack_Node launched, waiting for arm and RC command")
        self.rate = rospy.Rate(20)
    
    def state_cb(self, data):
        if self.was_armed and not data.armed:
            rospy.loginfo("Disarmed detected: preparing full reset for next flight")
            # Mark just disarmed, trigger reset on next arm
            self._just_disarmed = True
        
        prev_armed = self.was_armed
        self.was_armed = data.armed

        if not prev_armed and data.armed and self._just_disarmed:
            rospy.loginfo("Re-armed after disarm: resetting node state")
            self._reset_flight_cycle()
            threading.Timer(15.0, self._start_recording).start()
            self._just_disarmed = False
            
        self.current_state = data
    
    def pose_cb(self, data):
        self.current_pose = data
        if self.recording and not self.return_triggered and not self.is_channel18_triggered:
            if self.waypoints:
                last_wp = self.waypoints[-1]
                dx = data.pose.position.x - last_wp.position.x
                dy = data.pose.position.y - last_wp.position.y
                dz = data.pose.position.z - last_wp.position.z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist >= 0.3:  # Record every 0.3m
                    rospy.loginfo("Record new waypoints: (%.2f, %.2f, %.2f)",
                                  data.pose.position.x, data.pose.position.y, data.pose.position.z)
                    self.waypoints.append(data.pose)
            else:
                self.waypoints.append(data.pose)
    
    def rc_cb(self, data):
        self.last_rc_time = time.time()

        if self.rc_loss_detected:
            rospy.loginfo("RC signal restored")
            self.rc_loss_detected = False
            if self.rc_loss_trigger_thread is not None and self.rc_loss_trigger_thread.is_alive():
                self.rc_loss_trigger_thread = None
                rospy.loginfo("Delayed backtrack trigger cancelled")

        # Check if channel18 (index 17) is below 900
        if len(data.channels) >= 18:
            channel18_value = data.channels[17]  # Channel18 corresponds to index 17
            prev_channel18_failsafe = self.channel18_failsafe_detected
            self.channel18_failsafe_detected = channel18_value < 900

            # 发布RC状态：正常为True，通道18<900时为False
            status_msg = Bool()
            status_msg.data = not self.channel18_failsafe_detected  # 正常=True, 失控=False
            self.status_rc_loss_pub.publish(status_msg)

            if self.channel18_failsafe_detected and not prev_channel18_failsafe and not self.return_triggered and not self.is_channel18_triggered:
                rospy.logwarn("Channel18 failsafe detected! Value: %d (Channel18 < 900)", channel18_value)
                if self.channel18_failsafe_thread is None:
                    self.channel18_failsafe_thread = threading.Thread(target=self.channel18_failsafe_trigger)
                    self.channel18_failsafe_thread.daemon = True
                    self.channel18_failsafe_thread.start()
            elif not self.channel18_failsafe_detected and prev_channel18_failsafe:
                rospy.loginfo("Channel18 restored to normal level")
                self.channel18_failsafe_thread = None

        # Check channel7 (index 6) button logic
        if len(data.channels) >= 7:
            current_time = time.time()
            channel7_value = data.channels[6]
            channel7_currently_pressed = channel7_value < 1500

            # Button pressed (1187)
            if channel7_currently_pressed and not self.channel7_pressed:
                # If in backtrack mode, stop immediately
                if self.return_triggered or self.is_channel18_triggered:
                    # Breakpoint continuation: only reset return flags, keep waypoints and reversed state
                    self.return_triggered = False
                    self.is_channel18_triggered = False
                    self.hover_wait_counter = 0
                    self.hover_wait_complete = False
                    self.hover_target_pose = None
                    self.landing_hover_counter = 0
                    self.landing_hover_pose = None
                    self.recording = False
                    rospy.loginfo("Channel 7 button pressed - immediately stopping backtrack")
                    rospy.sleep(0.1)  # Wait one cycle to ensure flags take effect
                    try:
                        resp = self.set_mode_srv(custom_mode="POSCTL")
                        if resp.mode_sent:
                            rospy.loginfo("Switching to POSITION mode succeed")
                        else:
                            rospy.logwarn("Switching to POSITION mode failed")
                    except rospy.ServiceException as e:
                        rospy.logerr("Exception of calling set_mode service: %s", e)
                    

                    rospy.loginfo("Return paused - waypoints preserved for resume. Remaining waypoints: %d", len(self.waypoints))
                else:
                    self.channel7_pressed = True
                    self.channel7_press_start_time = current_time
                    self.channel7_on_triggered = False
                    self.channel7_off_triggered = False
                    rospy.loginfo("Channel 7 button pressed (value: %d)", channel7_value)


            # Button released (1874)
            elif not channel7_currently_pressed and self.channel7_pressed:
                self.channel7_pressed = False
                press_duration = current_time - self.channel7_press_start_time
                rospy.loginfo("Channel 7 button released after %.2f seconds", press_duration)

                # Reset trigger flags
                self.channel7_on_triggered = False
                self.channel7_off_triggered = False

            # Check hold duration while button is pressed
            elif channel7_currently_pressed and self.channel7_pressed:
                press_duration = current_time - self.channel7_press_start_time
                # Hold 2s - start backtrack
                if press_duration >= 2.0 and not self.channel7_on_triggered and not self.return_triggered and not self.is_channel18_triggered:
                    # 检查是否在AUTO.LAND模式，如果是则不触发返航
                    if self.current_state.mode == "AUTO.LAND":
                        rospy.loginfo("Currently in AUTO.LAND mode - backtrack cancelled, keeping one-key landing priority")
                        self.channel7_pressed = False  # Reset button state
                    else:
                        self.channel7_on_triggered = True
                        rospy.loginfo("Channel 7 held for 2+ seconds - starting backtrack")
                        self.return_triggered = True
                        self.start_return_home()
    
    def channel18_failsafe_trigger(self):
        """
        通道18故障安全触发 - 设置标志位，15秒悬停等待后开始返航
        如果当前在AUTO.LAND模式，不触发返航（一键降落优先级最高）
        """
        rospy.loginfo("Channel18 below 900 detected, will trigger backtrack in 1 seconds if not restored")
        start_wait = time.time()
        while time.time() - start_wait < 1 and not rospy.is_shutdown():
            if not self.channel18_failsafe_detected:
                return
            rospy.sleep(0.05)

        # 检查是否在AUTO.LAND模式，如果是则不触发返航
        if self.current_state.mode == "AUTO.LAND":
            rospy.loginfo("Currently in AUTO.LAND mode - backtrack cancelled, keeping one-key landing priority")
            return

        if self.channel18_failsafe_detected and not self.return_triggered and not self.is_channel18_triggered and self.current_state.armed:
            rospy.loginfo("Channel18 remained below 900 for more than 1 seconds, starting 15s hover wait before backtrack")
            self.is_channel18_triggered = True
            self.hover_wait_counter = 0
            self.hover_wait_complete = False
            self.hover_target_pose = None  # Reset hover target, will be set in publish_setpoints
    
    def check_rc_loss(self):
        """检查RC信号丢失的线程"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state.armed and time.time() - self.last_rc_time > 0.15:
                if not self.rc_loss_detected:
                    rospy.logwarn("RC signal loss detected")
                    self.rc_loss_detected = True
                    self.rc_loss_start_time = time.time()
                    if not self.return_triggered and not self.is_channel18_triggered and self.rc_loss_trigger_thread is None:
                        rospy.logwarn("failsafe_backtrack_trigger")
                        self.rc_loss_trigger_thread = threading.Thread(target=self.failsafe_backtrack_trigger)
                        self.rc_loss_trigger_thread.daemon = True
                        self.rc_loss_trigger_thread.start()
            rate.sleep()
    
    def failsafe_backtrack_trigger(self):
        """
        RC信号丢失超过0.1秒后触发返航
        如果当前在AUTO.LAND模式，不触发返航（一键降落优先级最高）
        """
        rospy.loginfo("RC loss detected, will trigger backtrack in 0.1 seconds if signal not restored")
        start_wait = time.time()
        while time.time() - start_wait < 0.1 and not rospy.is_shutdown():
            if not self.rc_loss_detected:
                return
            rospy.sleep(0.05)

        # 检查是否在AUTO.LAND模式，如果是则不触发返航
        if self.current_state.mode == "AUTO.LAND":
            rospy.loginfo("Currently in AUTO.LAND mode - backtrack cancelled, keeping one-key landing priority")
            return

        if self.rc_loss_detected and not self.return_triggered and not self.is_channel18_triggered and self.current_state.armed:
            rospy.loginfo("RC signal lost for more than 0.1 seconds, triggering backtrack")
            self.return_triggered = True
            self.start_return_home()
    
    def start_return_home(self):
        # Breakpoint continuation logic: only reverse waypoints on first trigger
        if not self.is_waypoints_reversed:
            rospy.loginfo("First time return triggered - reversing waypoints for backtrack")
            # self.waypoints.reverse()
            self.is_waypoints_reversed = True
            rospy.loginfo("Total waypoints to backtrack: %d", len(self.waypoints))
        else:
            rospy.loginfo("Resume return triggered - continuing with remaining %d waypoints", len(self.waypoints))
        self.pre_offboard_counter = 0
            
    def set_position_mode(self):
        try:
            resp = self.set_mode_srv(custom_mode="POSCTL")
            if resp.mode_sent:
                rospy.loginfo("Switching POSCTL succeed")
            else:
                rospy.logwarn("Switching to POSCTL failed")
        except rospy.ServiceException as e:
            rospy.logerr("Exception of calling set_mode service: %s", e)
    
    def set_offboard_mode(self):
        try:
            resp = self.set_mode_srv(custom_mode="OFFBOARD")
            if resp.mode_sent:
                rospy.loginfo("Switching OFFBOARD succeed")
            else:
                rospy.logwarn("Switching to OFFBOARD failed")
        except rospy.ServiceException as e:
            rospy.logerr("Exception of calling set_mode service: %s", e)
            
    def set_auto_land_mode(self):
        try:
            resp = self.set_mode_srv(custom_mode="AUTO.LAND")
            if resp.mode_sent:
                rospy.loginfo("Switching to AUTO.LAND succeed - Starting automatic landing")
            else:
                rospy.logwarn("Switching to AUTO.LAND failed")
        except rospy.ServiceException as e:
            rospy.logerr("Exception of calling set_mode service: %s", e)
    
    def clear_waypoints(self):
        """完全清空航点和重置所有状态（用于返航完成）"""
        self.waypoints = []
        self.recording = True
        self.return_triggered = False
        self.is_waypoints_reversed = False  # Reset reversed flag
        rospy.loginfo("All waypoints cleared - ready for new recording")

    def _reset_flight_cycle(self):
        """重置所有飞行周期相关的状态"""
        self.waypoints = []
        self.recording = False
        self.return_triggered = False
        self.is_waypoints_reversed = False  # Reset reversed flag
        self.rc_loss_detected = False
        self.channel18_failsafe_detected = False
        self.rc_loss_trigger_thread = None
        self.channel18_failsafe_thread = None

        # Reset channel7 button state
        self.channel7_pressed = False
        self.channel7_press_start_time = 0
        self.channel7_on_triggered = False
        self.channel7_off_triggered = False

        # Reset channel18 wait state
        self.is_channel18_triggered = False
        self.hover_wait_counter = 0
        self.hover_wait_complete = False
        self.hover_target_pose = None

        # Reset landing hover state
        self.landing_hover_counter = 0
        self.landing_hover_pose = None

    def _start_recording(self):
        """真正开启 waypoint 录制，并立刻把当前位置作为首点"""
        rospy.loginfo("Started recording waypoints")
        self.recording = True
        if self.current_pose:
            self.waypoints = [self.current_pose.pose]

    
    def publish_setpoints(self):
        """
        发布航点进行返航，到达航点后删除该航点（断点续航核心逻辑）
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # 检查是否在AUTO.LAND模式，如果是则跳过所有返航逻辑
            if self.current_state.mode == "AUTO.LAND":
                # 在AUTO.LAND模式下，清除返航相关的标志位，但不做任何动作
                if self.return_triggered or self.is_channel18_triggered:
                    rospy.loginfo("AUTO.LAND mode detected - clearing backtrack flags, not interfering with landing")
                    self.return_triggered = False
                    self.is_channel18_triggered = False
                    self.hover_wait_counter = 0
                    self.hover_wait_complete = False
                    self.hover_target_pose = None
                    self.landing_hover_counter = 0
                    self.landing_hover_pose = None
                rate.sleep()
                continue

            # Channel18 hover wait state
            if self.is_channel18_triggered and not self.hover_wait_complete:
                if self.current_pose is not None:
                    # On first entry, capture current position as fixed hover target
                    if self.hover_target_pose is None:
                        self.hover_target_pose = PoseStamped()
                        self.hover_target_pose.header.frame_id = "odom"
                        self.hover_target_pose.pose = self.current_pose.pose
                        rospy.loginfo("Hover target set at: (%.2f, %.2f, %.2f)",
                                      self.hover_target_pose.pose.position.x,
                                      self.hover_target_pose.pose.position.y,
                                      self.hover_target_pose.pose.position.z)

                    # Publish fixed hover target
                    self.hover_target_pose.header.stamp = rospy.Time.now()
                    self.setpoint_pub.publish(self.hover_target_pose)

                    self.hover_wait_counter += 1

                    if self.hover_wait_counter > 10 and self.current_state.mode != "OFFBOARD":
                        self.set_offboard_mode()

                    # 15 seconds = 15 * 20Hz = 300 cycles
                    if self.hover_wait_counter >= 300:
                        self.hover_wait_complete = True
                        rospy.loginfo("15s hover wait completed, starting backtrack")
                        self.return_triggered = True
                        self.is_channel18_triggered = False
                        self.hover_target_pose = None  # Clear hover target
                        self.start_return_home()
                    elif self.hover_wait_counter % 20 == 0:  # Output every second
                        remaining_seconds = 15 - (self.hover_wait_counter // 20)
                        rospy.loginfo("Channel18 hover wait: %d seconds remaining", remaining_seconds)
                        
            elif self.return_triggered:
                if self.current_state.mode != "OFFBOARD":
                    # Send current position as target
                    if self.current_pose is not None:
                        target_pose = PoseStamped()
                        target_pose.header.stamp = rospy.Time.now()
                        target_pose.header.frame_id = "odom"
                        target_pose.pose = self.current_pose.pose
                        self.setpoint_pub.publish(target_pose)

                        self.pre_offboard_counter += 1

                        # Switch mode after sending more than 10 times
                        if self.pre_offboard_counter > 10:
                            self.set_offboard_mode()
                else:
                    if len(self.waypoints) > 0:
                        # Always use first waypoint as target (will be deleted upon arrival)
                        target_waypoint = self.waypoints[-1]  # Last element in list
                        
                        target_pose = PoseStamped()
                        target_pose.header.stamp = rospy.Time.now()
                        target_pose.header.frame_id = "odom"

                        if self.current_pose is not None:
                            # Calculate distance from current position to target waypoint
                            dx = target_waypoint.position.x - self.current_pose.pose.position.x
                            dy = target_waypoint.position.y - self.current_pose.pose.position.y
                            dz = target_waypoint.position.z - self.current_pose.pose.position.z
                            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

                            # Limit maximum movement distance per cycle
                            max_distance_per_cycle = 0.65

                            if distance > max_distance_per_cycle:
                                # Interpolate intermediate point
                                ratio = max_distance_per_cycle / distance
                                target_pose.pose.position.x = self.current_pose.pose.position.x + dx * ratio
                                target_pose.pose.position.y = self.current_pose.pose.position.y + dy * ratio
                                target_pose.pose.position.z = self.current_pose.pose.position.z + dz * ratio
                                # Use target waypoint's orientation to maintain recorded heading
                                target_pose.pose.orientation = target_waypoint.orientation
                            else:
                                # Close distance, directly use target waypoint (including orientation)
                                target_pose.pose = target_waypoint

                            self.setpoint_pub.publish(target_pose)

                            if distance < 0.2:  # Waypoint arrival distance threshold
                                # Breakpoint continuation core: delete reached waypoint
                                reached_point = self.waypoints.pop()
                                rospy.loginfo("Waypoint reached and removed. Remaining waypoints: %d", len(self.waypoints))
                                rospy.loginfo("Reached point: (%.2f, %.2f, %.2f)",
                                            reached_point.position.x, reached_point.position.y, reached_point.position.z)
                        else:
                            # If no current position info, directly publish target waypoint (including orientation)
                            target_pose.pose = target_waypoint

                            self.setpoint_pub.publish(target_pose)
                            
                    else:
                        # All waypoints reached - hover 5s before landing
                        if self.current_pose is not None:
                            # On first entry, capture current position as pre-landing hover point
                            if self.landing_hover_pose is None:
                                self.landing_hover_pose = PoseStamped()
                                self.landing_hover_pose.header.frame_id = "odom"
                                self.landing_hover_pose.pose = self.current_pose.pose
                                rospy.loginfo("All waypoints reached - Hovering at final position for 5 seconds before landing")
                                rospy.loginfo("Landing hover target: (%.2f, %.2f, %.2f)",
                                              self.landing_hover_pose.pose.position.x,
                                              self.landing_hover_pose.pose.position.y,
                                              self.landing_hover_pose.pose.position.z)

                            # Publish fixed pre-landing hover point
                            self.landing_hover_pose.header.stamp = rospy.Time.now()
                            self.setpoint_pub.publish(self.landing_hover_pose)

                            self.landing_hover_counter += 1

                            # Output countdown every second
                            if self.landing_hover_counter % 20 == 0:
                                remaining_seconds = 5    - (self.landing_hover_counter // 20)
                                if remaining_seconds > 0:
                                    rospy.loginfo("Landing in %d seconds...", remaining_seconds)

                            # 5 seconds = 5 * 20Hz = 100 cycles
                            if self.landing_hover_counter >= 100:
                                rospy.loginfo("Landing hover completed, initiating automatic landing")
                                self.set_position_mode()
                                rospy.sleep(1.0)
                                self.set_auto_land_mode()
                                self.clear_waypoints()  # Clear all state
                                self.landing_hover_counter = 0
                                self.landing_hover_pose = None
            rate.sleep()

if __name__ == '__main__':
    try:
        node = BacktrackNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
