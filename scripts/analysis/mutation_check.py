#!/usr/bin/env python3
"""错误注入检查：对实现注入典型错误，确认单测能发现，用来检验测试本身的判别力。

在 scripts/docker/run_tests.sh 留下的容器中运行（由 run_mutation_check.sh 调用），只在 /tmp/mutation
中操作源码副本，不改仓库。每个注入把一段原文替换掉，单独编译该组件和它的单测并运行，有用例失败
即为"已发现"。UseCase 与已编译的库一起链接，注入后的定义覆盖库中的同名定义。原文必须恰好出现
指定次数；代码改动后匹配不到时报告"原文不匹配"，需要同步更新下表。
用法：mutation_check.py [组件或注入名 ...]；全部发现且全部匹配时退出码为 0。
"""
import os
import re
import shutil
import subprocess
import sys

SRC = "/ws/src/px4_backtrack_optimised"
WORK = "/tmp/mutation"
GTEST_INC = "/usr/src/googletest/googletest/include"
GTEST_LIB = "/ws/build/gtest/lib"
ROS_FLAGS = ["-I/ws/src/trajectory_core/include", "-I/opt/ros/noetic/include", "-L/ws/devel/lib",
             "-L/opt/ros/noetic/lib", "-lpx4_backtrack_optimised_lib", "-ltrajectory_core", "-lroscpp",
             "-lrosconsole", "-lrostime", "-Wl,-rpath,/ws/devel/lib:/opt/ros/noetic/lib"]

# 组件：(参与编译的源文件，相对 src/；tests/unit 下的单测；是否链接 ROS 和已编译的库)
COMPONENTS = {
    "link_monitor": (["domain/link_monitor.cpp"], ["test_link_monitor.cpp"], False),
    "pilot_input": (["domain/pilot_input.cpp", "domain/link_monitor.cpp"], ["test_pilot_input.cpp"], False),
    "battery_monitor": (["domain/battery_monitor.cpp"], ["test_battery_monitor.cpp"], False),
    "airborne_monitor": (["domain/airborne_monitor.cpp"], ["test_airborne_monitor.cpp"], False),
    "recording_buffer": (["domain/recording_buffer.cpp"], ["test_recording_buffer.cpp"], False),
    "return_state_machine": (["domain/return_state_machine.cpp"],
                             ["test_return_state_machine.cpp", "test_return_state_machine_invariants.cpp"],
                             False),
    "backtrack_usecase": (["application/backtrack_usecase.cpp"],
                          ["test_main.cpp", "test_backtrack_usecase.cpp"], True),
    "planning_worker": (["application/planning_worker.cpp"],
                        ["test_main.cpp", "test_planning_worker.cpp"], True),
    "tick_report": (["application/tick_report.cpp"], ["test_main.cpp", "test_backtrack_usecase.cpp"], True),
    "debug_log": (["infrastructure/debug_log.cpp"], ["test_main.cpp", "test_debug_log.cpp"], True),
}

# (名称, 组件, 原文, 替换, 说明[, 原文出现次数，默认 1])
MUTATIONS = [
    ('bucket_no_cap', 'link_monitor', 'std::min(status_.loss_bucket_s + dt, config_.ch18_confirm_s)', 'status_.loss_bucket_s + dt', '漏桶没有上限（R-55）'),
    ('no_stall_guard', 'link_monitor', 't - last_update_t_ + kTimeEpsS >= config_.rc_loss_timeout_s', 'false', '本模块卡顿当作 RC 超时（R-61）'),
    ('recover_without_fresh_frames', 'link_monitor', 'if (status_.lost && frames_fresh &&', 'if (status_.lost &&', '没有新帧也判定恢复'),
    ('no_ch18_gating', 'pilot_input', '  if (link.ch18_low_now) {\n    cancelPress();\n    return;\n  }\n', '', '失控帧中的 CH7 也当作有效输入（S2）'),
    ('no_wait_release', 'pilot_input', 'if (!status_.wait_release) {', 'if (true) {', '恢复后不等松开就接受按下（S3）'),
    ('no_degraded_sampling', 'pilot_input', 'press_link_clean_ = press_link_clean_ && !link.degraded;', '(void)0;', '按下期间不检查退化', 2),
    ('battery_any_value_valid', 'battery_monitor', 'return std::isfinite(percentage) && percentage >= 0.0 && percentage <= 1.0;', 'return true;', '任意电量读数都当作有效'),
    ('battery_invalid_resets_run', 'battery_monitor', 'status_.invalid = true;', 'status_.invalid = true; confirming_ = false;', '无效读数打断低电量确认'),
    ('airborne_ignores_height', 'airborne_monitor', 'height_ok = status_.height_above_arm_m > config_.airborne_min_height_m;', 'height_ok = true;', '在空中判定不看高度（D23）'),
    ('airborne_no_stale_check', 'airborne_monitor', 'has_landed_ && t - landed_t_ < config_.fcu_state_stale_s', 'has_landed_', '落地状态过期仍然采信'),
    ('midair_restart_uses_current_height', 'airborne_monitor', 'status_.arm_height = ArmHeightRef::NotApplicable;', 'z_arm_ = z_; status_.arm_height = ArmHeightRef::Known;', '空中重启把当前高度当作解锁高度（R-58）'),
    ('gap_scan_ignored', 'recording_buffer',
     '         (points_[begin].position - points_[begin - 1].position).norm() <= max_gap_m) {',
     '         max_gap_m > 0.0) {', '断点扫描失效，规划跨过大间隔（D30）'),
    ('cursor_window_ignored', 'recording_buffer', '    if (arc > window_m) {\n      break;\n    }\n', '',
     '游标搜索不限窗口，路径自交时跳到另一段'),
    ('cursor_tie_larger_index', 'recording_buffer', '    if (distance <= best_distance) {',
     '    if (distance < best_distance) {', '距离相同时取更大的索引'),
    ('resume_not_shifted', 'recording_buffer',
     '    time_origin_ = odom_stamp - timestamp;\n', '',
     '续录后时间戳不接续（R-77）'),
    ('anchor_includes_end', 'recording_buffer', '  for (size_t i = 0; i < end; ++i) {',
     '  for (size_t i = 0; i <= end && i < points_.size(); ++i) {', '找点接上的候选包含游标所在的点（D33）'),
    ('anchor_earliest_not_nearest', 'recording_buffer',
     '(!best || distance_sq < best_distance_sq)', '!best', '找点接上取最早的点而不是最近的（R-84）'),
    ('no_landed_rule', 'return_state_machine', '  if (landed) {\n    enterHandover(c, HandoverReason::MissionLanded, true, "3", out);\n    return;\n  }', '', '去掉任务中落地保护（规则 3）'),
    ('start_on_ground', 'return_state_machine', '  if (!c.airborne.airborne) {\n    trigger_ignored_landed_ = true;', '  if (false) {\n    trigger_ignored_landed_ = true;', '启动任务不检查在空中（D23）'),
    ('gap_is_takeover', 'return_state_machine', '      if (c.stream_gap_recent) {', '      if (false) {', '自身断流当作飞手接管（D22）'),
    ('switch_out_always_exits', 'return_state_machine', '  if (linkClean(c)) {\n    exitMission(c, rule, "EXTERNAL_MODE", out);', '  if (true) {\n    exitMission(c, rule, "EXTERNAL_MODE", out);', '链路丢失时被切出也退出（D2）'),
    ('handover_ignores_link_loss', 'return_state_machine', '  if (c.link.lost) {\n    handover_direct_ = true;\n  }', '', '交接中链路丢失仍经过 POSCTL'),
    ('handover_entry_ignores_link_loss', 'return_state_machine', '  handover_direct_ = direct || c.link.lost;', '  handover_direct_ = direct;', '进入交接时不看链路'),
    ('exited_posctl_when_lost', 'return_state_machine', '        if (!c.link.lost) {\n          out.mode_intent = ModeIntent::Posctl;', '        if (true) {\n          out.mode_intent = ModeIntent::Posctl;', '退出后链路丢失仍请求 POSCTL（R-67）'),
    ('exit_ignores_link_ok', 'return_state_machine', '  if (!request.link_ok) {', '  if (false) {', '短按退出不看链路'),
    ('exit_ignores_start_time', 'return_state_machine', '  if (request.press_start_t <= mission_.start_t) {', '  if (false) {', '短按不看按下时间（S5）'),
    ('takeover_4c_when_degraded', 'return_state_machine', '      if (linkClean(c) && c.mode != mission_.start_mode) {', '      if (c.mode != mission_.start_mode) {', '确认前接管不看链路（P4）'),
    ('takeover_4c_ignores_start_mode', 'return_state_machine', '      if (linkClean(c) && c.mode != mission_.start_mode) {', '      if (linkClean(c)) {', '确认前把开始时的模式也当作接管'),
    ('reclaim_link_clean_ignored', 'return_state_machine', '      // 链路先恢复：飞手当时的模式选择生效（R-63）\n      if (linkClean(c)) {', '      if (false) {', '重新申请中链路恢复也不退出（R-63）'),
    ('fcu_landing_always_rerequest', 'return_state_machine', '      if (c.link.lost && knownMode() != kModeAutoLand) {\n        out.mode_intent', '      if (knownMode() != kModeAutoLand) {\n        out.mode_intent', 'AutoLandYield 链路正常也重新请求'),
    ('exit_hold_not_captured', 'return_state_machine', '  exit_position_ = c.position;', '', '退出保持点没有记录（原点）'),
    ('handover_reason_none', 'return_state_machine', '  handover_reason_ = reason;', '  handover_reason_ = HandoverReason::None;', '交接没有原因（SC9）'),
    ('stale_mode_decides', 'return_state_machine', '  if (c.fcu_state_fresh && updateControl(c, out)) {', '  if (updateControl(c, out)) {', '飞控状态中断时仍按模式判定（R-62）'),
    ('follow_not_frozen', 'return_state_machine', '  // 不持有控制权或里程计延迟时冻结进度（R-18）\n  if (!owned) {\n    return;\n  }', '', '跟随不因失去控制权冻结'),
    ('reclaim_without_offset_check', 'return_state_machine', '  if (!yaw_ok || !offset_ok) {', '  if (!yaw_ok) {', '重新接入不检查偏差（B3）'),
    ('preframes_skipped', 'return_state_machine', '      if (pre_frames_ < config_.pre_offboard_setpoint_count) {', '      if (false) {', '不预发直接申请 OFFBOARD'),
    ('pending_never_lands', 'return_state_machine', '  if (c.now - pending_.start_t + kTimeEpsS >= config_.return_pending_max_s) {', '  if (false) {', '补点等待没有上限'),
    ('battery_starts_from_exited', 'return_state_machine', '  const bool battery_low = !from_exited && c.battery.low;', '  const bool battery_low = c.battery.low;', '退出后低电量也启动返航（D1）'),
    ('hover_skipped', 'return_state_machine', '  if (pending_.link_lost) {\n    hover_start_t_ = c.now;', '  if (false) {\n    hover_start_t_ = c.now;', '失控不先悬停（D16）'),
    ('record_in_mission', 'return_state_machine', '  out.follow_active = phase_ == MissionPhase::Follow &&', '  out.record = true;\n  out.follow_active = phase_ == MissionPhase::Follow &&', '任务中继续录制'),
    ('reclaim_hold_not_tracking', 'return_state_machine',
     '    if (knownMode() != kModeOffboard) {\n      reclaim_position_ = c.position;\n      reclaim_yaw_ = c.yaw;\n    }\n',
     '', '重新申请期间保持切出时的位置，拿回 OFFBOARD 后飞回去（D30）'),
    ('reclaim_keeps_old_hold', 'return_state_machine',
     '    trigger_position_ = reclaim_position_;\n    arrival_position_ = reclaim_position_;\n', '',
     '悬停、准备、到点阶段重新接入后飞回原来的保持点（D30）'),
    ('prepare_ignores_offset', 'return_state_machine',
     '  // 开始跟随前检查偏差：离路径超过安全距离时能找点接上就接上，否则原地保持（D30、D33）\n  if (!offsetOk(c)) {',
     '  // 开始跟随前检查偏差：离路径超过安全距离时能找点接上就接上，否则原地保持（D30、D33）\n  if (false) {',
     '开始跟随前不检查偏差（D30）'),
    ('follow_offset_rejoins', 'return_state_machine',
     '  // 否则原地保持、冻结进度（D30）\n  if (!offsetOk(c)) {',
     '  // 否则原地保持、冻结进度（D30）\n  if (false) {',
     '跟随中偏差超过安全距离仍回归（D30）'),
    ('gap_arrival_lands', 'return_state_machine',
     '  if (!c.trail_gap) {\n    enterHandover(c, HandoverReason::Arrived, false, "7", out);',
     '  if (true) {\n    enterHandover(c, HandoverReason::Arrived, false, "7", out);', '到达轨迹断点也降落（D30）'),
    ('gap_too_close_lands', 'return_state_machine',
     '      if (!c.trail_gap) {\n        enterHandover(c, c.plan_status == PlanStatus::FailedTooClose',
     '      if (true) {\n        enterHandover(c, c.plan_status == PlanStatus::FailedTooClose',
     '有断点时点过近、航点降级失败也降落（D30）'),
    ('hover_ends_on_plan_ready', 'return_state_machine',
     '      if (c.now - hover_start_t_ + kTimeEpsS >= config_.ch18_hover_wait_s) {',
     '      if (planReady(c) || c.now - hover_start_t_ + kTimeEpsS >= config_.ch18_hover_wait_s) {',
     '规划就绪就提前结束失控悬停（D3、R-17）'),
    ('offboard_gate_removed', 'return_state_machine',
     '      } else if (!linkClean(c) || !c.fcu_state_fresh ||\n                 c.fcu_state_t > pre_frames_done_t_) {',
     '      } else {', '预发结束后不等飞控状态就申请 OFFBOARD（R-75）'),
    ('reanchor_without_anchor', 'return_state_machine',
     '    if (owned && c.anchor_ahead) {\n      reanchor(c, "7", "PATH_OFFSET", out);',
     '    if (owned) {\n      reanchor(c, "7", "PATH_OFFSET", out);', '离路径超过安全距离时没找到点也重新规划（D33）'),
    ('reanchor_not_owned', 'return_state_machine',
     '    if (owned && c.anchor_ahead) {\n      reanchor(c, "7", "PATH_OFFSET", out);',
     '    if (c.anchor_ahead) {\n      reanchor(c, "7", "PATH_OFFSET", out);', '不持有控制权、里程计延迟时也找点接上（D33）'),
    ('gap_reanchor_any_anchor', 'return_state_machine',
     '  } else if (control_ == ControlState::Owned && !c.odom_stale && c.anchor_past_gap) {',
     '  } else if (control_ == ControlState::Owned && !c.odom_stale && c.anchor_ahead) {',
     '停在断点处也接断点之后的点，白白重规划（D33）'),
    ('reanchor_yaw_from_old_plan', 'return_state_machine',
     '      if (planReady(c) && c.path_start_yaw && !out.commands.reanchor) {',
     '      if (planReady(c) && c.path_start_yaw) {', '找点接上的那一拍用旧轨迹的起始切向（D33）'),
    ('reclaim_reanchor_keeps_reclaiming', 'return_state_machine',
     '    setControl(ControlState::Owned, "4", "RECLAIMED", out);\n    reanchor(c, "4", "PATH_OFFSET", out);',
     '    reanchor(c, "4", "PATH_OFFSET", out);', '重新申请中找点接上后仍处于重新申请（D33）'),
    ('reanchor_hold_not_moved', 'return_state_machine',
     '  trigger_position_ = c.position;\n  trigger_yaw_ = c.yaw;\n  prepare_wait_s_ = 0.0;\n',
     '  prepare_wait_s_ = 0.0;\n', '找点接上后飞回原来的触发点（D30、D33）'),
    ('escalate_without_low_battery', 'return_state_machine',
     '  if (c.battery.low && waitTimedOut()) {\n    enterHandover(c, HandoverReason::Escalated',
     '  if (waitTimedOut()) {\n    enterHandover(c, HandoverReason::Escalated', '电量正常时超时也降落（D11）'),
    ('waits_count_while_odom_stale', 'return_state_machine',
     '  if (c.odom_stale) {\n    return;\n  }\n  const double dt = std::max(0.0, c.now - prev_.now);',
     '  const double dt = std::max(0.0, c.now - prev_.now);', '里程计延迟时也计时，里程计中断会引起降落（D31）'),
    ('prepare_counts_while_pending', 'return_state_machine',
     '      c.plan_status != PlanStatus::None && c.plan_status != PlanStatus::Pending) {',
     '      c.plan_status != PlanStatus::None) {', '规划中也计准备时间'),
    ('stall_ignores_progress', 'return_state_machine',
     '    if (c.remaining_m <= stall_mark_m_ - config_.stall_min_progress_m) {', '    if (false) {',
     '剩余路径缩短也不重新计停滞'),
    ('stall_not_reset_on_follow', 'return_state_machine',
     '  // 进入跟随：停滞重新计时\n  stall_wait_s_ = 0.0;\n  stall_mark_m_ = c.remaining_m;\n', '',
     '重新进入跟随时停滞不归零'),
    ('reclaim_wait_not_reset', 'return_state_machine', '  reclaim_wait_s_ = 0.0;\n', '',
     '每次重新申请不重新计时'),
    ('prepare_not_reset_on_reanchor', 'return_state_machine',
     '  trigger_yaw_ = c.yaw;\n  prepare_wait_s_ = 0.0;\n  addEvent(out, EventType::Mission, "reanchored"',
     '  trigger_yaw_ = c.yaw;\n  addEvent(out, EventType::Mission, "reanchored"', '找点接上后准备计时不归零'),
    ('prepare_alignment_never_given_up', 'return_state_machine',
     '  if (!aligned && !(c.battery.low && waitTimedOut())) {', '  if (!aligned) {',
     '低电量准备超时仍等对准，返航开始不了（D32）'),
    ('arrival_hover_counts_without_control', 'return_state_machine',
     '  if (phase_ == MissionPhase::Arrival && control_ == ControlState::Owned) {\n    arrival_owned_s_ += dt;',
     '  if (phase_ == MissionPhase::Arrival) {\n    arrival_owned_s_ += dt;', '到点悬停不持有控制权也计时'),
    ('gap_hold_never_escalates', 'return_state_machine',
     '    case MissionPhase::Follow:\n    case MissionPhase::Arrival:\n      // 到点悬停也计停滞',
     '    case MissionPhase::Follow:\n      // 到点悬停也计停滞', '停在断点处找不到点时低电量也不降落（R-85）'),
    ('odom_nonfinite_accepted', 'backtrack_usecase',
     '  if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(yaw)) {\n    return;\n  }\n',
     '', '非有限的里程计也采信（R-73）'),
    ('disconnected_state_used', 'backtrack_usecase', '  if (!connected) {\n    return;\n  }\n', '',
     '未连上飞控时的状态也采信'),
    ('fcu_stale_ignored', 'backtrack_usecase',
     '  c.fcu_state_fresh = has_fcu_state_ &&\n                      now - fcu_state_t_ <= config_.fcu_state_stale_s;\n',
     '  c.fcu_state_fresh = has_fcu_state_;\n', '飞控状态中断不判定'),
    ('probe_stale_projection', 'backtrack_usecase',
     '    c.projection_ok = trajectory_session_->probe(position_, c.cross_track_m);\n',
     '    c.projection_ok = trajectory_session_->hasValidProjection();\n'
     '    c.cross_track_m = trajectory_session_->getCrossTrackError();\n',
     '重新接入的偏差沿用冻结时的投影（R-74）'),
    ('publishing_not_ended', 'backtrack_usecase', '    // 不发布即结束这一段（R-69）\n    publishing_ = false;\n', '',
     '不发布的拍不结束这一段，任务之间的空档算作断流（R-69）'),
    ('min_points_ignored', 'backtrack_usecase',
     '  c.has_min_points = recording_buffer_.size() >=\n                     static_cast<size_t>(config_.record_min_points);\n',
     '  c.has_min_points = true;\n', '不检查录制点数'),
    ('record_during_mission', 'backtrack_usecase',
     '  if (record_) {\n    recording_buffer_.addPoint(position, yaw, stamp);\n  }\n',
     '  recording_buffer_.addPoint(position, yaw, stamp);\n', '任务期间也录制'),
    ('planning_timeout_ignored', 'backtrack_usecase',
     '  if (elapsed + kTimeEpsS >= config_.planning_timeout_s) {', '  if (false) {',
     '规划超时不改为航点降级（S16）'),
    ('planning_slow_not_cleared', 'backtrack_usecase',
     '  c.planning_slow = plan_status_ == domain::PlanStatus::Pending &&\n'
     '                    now - plan_submit_t_ + kTimeEpsS >= config_.planning_warn_s;\n',
     '  c.planning_slow = now - plan_submit_t_ + kTimeEpsS >= config_.planning_warn_s;\n',
     '规划完成后规划慢的告警不清除'),
    ('fallback_offset_zero', 'backtrack_usecase',
     '    c.cross_track_m = (position_ - waypoint_fallback_session_.currentTarget().position).norm();\n',
     '    c.cross_track_m = 0.0;\n', '航点降级偏差按 0 提供，重新接入不检查偏差（D30）'),
    ('plan_crosses_gap', 'backtrack_usecase',
     '  plan_begin_ = recording_buffer_.continuousFrom(config_.state_machine_config.safe_distance_m);',
     '  plan_begin_ = 0;', '规划用整段录制，跨过断点直线飞行（D30）'),
    ('trail_cut_ignored', 'backtrack_usecase', '    cutTrail(cursor_, events);\n', '',
     '退出时不截断，新任务先飞回原触发点（S11）'),
    ('cursor_not_updated', 'backtrack_usecase',
     '  cursor_ = recording_buffer_.nearestBefore(cursor_, reference, config_.cursor_window_m);\n',
     '  (void)reference;\n', '游标不随返航进度移动'),
    ('resume_not_called', 'backtrack_usecase', '    recording_buffer_.resume();\n', '',
     '录制恢复时时间戳不接续（R-77）'),
    ('anchor_ignores_gap', 'backtrack_usecase', '    c.anchor_past_gap = findAnchor(plan_begin_).has_value();\n',
     '    c.anchor_past_gap = c.anchor_ahead;\n', '停在断点处也接断点之后的点（D33）'),
    ('reanchor_gap_not_first', 'backtrack_usecase',
     '    const size_t anchor = past_gap ? *past_gap : *findAnchor(cursor_);\n',
     '    const size_t anchor = *findAnchor(cursor_);\n', '找点接上不优先接断点之前的点（R-84）'),
    ('reanchor_not_cut', 'backtrack_usecase', '    cutTrail(anchor, events);\n', '    (void)anchor;\n',
     '找点接上不截断，仍从原来的末点规划（D33）'),
    ('remaining_traj_constant', 'backtrack_usecase',
     '    c.remaining_m = trajectory_session_->getTrajectory()->pathLength() -\n'
     '                    trajectory_session_->getProjectedArcLength();\n',
     '    c.remaining_m = 1.0;\n', '轨迹的剩余路径不随进度缩短，正常返航也算停滞'),
    ('remaining_fallback_constant', 'backtrack_usecase',
     '    c.remaining_m = waypoint_fallback_session_.remainingLength();\n', '    c.remaining_m = 1.0;\n',
     '航点降级的剩余路径不随进度缩短'),
    ('cancel_keeps_running_result', 'planning_worker',
     'void PlanningWorker::cancel() {\n  std::lock_guard<std::mutex> lock(mutex_);\n  ++generation_;\n',
     'void PlanningWorker::cancel() {\n  std::lock_guard<std::mutex> lock(mutex_);\n',
     '取消后正在运行的求解仍交出结果'),
    # R7：上报与调试日志
    ('plan_input_only_when_ready', 'planning_worker',
     '  if (input.dumper) {\n    input.dumper->dumpInput(',
     '  if (input.dumper && result->status == domain::PlanStatus::Ready) {\n    input.dumper->dumpInput(',
     '规划输入只在轨迹就绪时导出，失败时没有输入（R-88）'),
    ('status_alerts_missing', 'tick_report', '    if (out.alerts.has(alert)) {', '    if (false) {',
     '状态话题不上报告警（S22）'),
    ('trace_field_missing', 'tick_report', '  f("plan_status", c.plan_status);\n', '',
     '逐拍记录漏掉一个条件字段，回放时取默认值（S31）'),
    ('trace_to_events_file', 'debug_log', '(line.trace ? trace_file_ : file_)', 'file_',
     '逐拍记录写进 events.jsonl'),
    ('keep_runs_counts_current', 'debug_log',
     '  const bool other_run = name.rfind("run_", 0) == 0 && name != current.filename().string();',
     '  const bool other_run = name.rfind("run_", 0) == 0;',
     '保留时把本次的目录当作旧目录，时钟不准时会删掉本次的目录'),
    ('keep_runs_off_by_one', 'debug_log',
     'keep_runs > 1 ? static_cast<std::size_t>(keep_runs - 1) : 0', 'static_cast<std::size_t>(keep_runs)',
     '保留个数不含本次，多留一个'),
]


def build_and_run(component, src_dir):
    """编译组件和它的单测并运行，返回 (是否编译成功, 失败的用例, 违反的不变量)。"""
    sources, tests, with_ros = COMPONENTS[component]
    exe = os.path.join(WORK, "test_" + component)
    main = [] if with_ros else [os.path.join(WORK, "gtest_main.cpp")]
    cmd = (["g++", "-std=c++17", "-O1", "-I" + os.path.join(WORK, "include"), "-I" + GTEST_INC,
            "-I/usr/include/eigen3"]
           + [os.path.join(src_dir, s) for s in sources]
           + [os.path.join(SRC, "tests/unit", t) for t in tests] + main
           + (ROS_FLAGS if with_ros else [])
           + ["-L" + GTEST_LIB, "-lgtest", "-lpthread", "-Wl,-rpath," + GTEST_LIB, "-o", exe])
    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0:
        return False, [line for line in build.stderr.splitlines() if "error" in line][:2], []
    # ROS 库的间接依赖不随 RUNPATH 查找，运行时补上库路径
    env = dict(os.environ, LD_LIBRARY_PATH="/ws/devel/lib:/opt/ros/noetic/lib")
    run = subprocess.run([exe], capture_output=True, text=True, timeout=900, env=env)
    # 没有 gtest 汇总行说明测试没有跑起来，不能当作通过
    if "[==========]" not in run.stdout:
        return False, ["运行失败：" + (run.stderr.strip().splitlines() or ["无输出"])[0]], []
    failed = sorted(set(re.findall(r"^\[  FAILED  \] \S+\.(\S+)", run.stdout, re.M)))
    invariants = sorted(set(re.findall(r"((?:SC\d+|STRUCT)[^:\n]*): \d+ 次", run.stdout)))
    return True, failed, invariants


def main():
    selected = set(sys.argv[1:])
    mutations = [m for m in MUTATIONS if not selected or m[0] in selected or m[1] in selected]
    shutil.rmtree(WORK, ignore_errors=True)
    os.makedirs(os.path.join(WORK, "src"))
    shutil.copytree(os.path.join(SRC, "include"), os.path.join(WORK, "include"))
    with open(os.path.join(WORK, "gtest_main.cpp"), "w") as f:
        f.write("#include <gtest/gtest.h>\nint main(int argc, char** argv) {"
                " ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }\n")

    def reset_sources():
        shutil.rmtree(os.path.join(WORK, "src"))
        shutil.copytree(os.path.join(SRC, "src"), os.path.join(WORK, "src"))

    print("== 基线（未注入）", flush=True)
    reset_sources()
    baseline_ok = True
    for component in sorted({m[1] for m in mutations}):
        ok, failed, _ = build_and_run(component, os.path.join(WORK, "src"))
        print(f"{component}: {'全部通过' if ok and not failed else '基线失败 ' + ' '.join(failed)}", flush=True)
        baseline_ok = baseline_ok and ok and not failed

    print("== 注入错误", flush=True)
    caught = mismatched = 0
    for name, component, old, new, desc, *rest in mutations:
        count = rest[0] if rest else 1
        reset_sources()
        path = os.path.join(WORK, "src", COMPONENTS[component][0][0])
        text = open(path, encoding="utf-8").read()
        if text.count(old) != count:
            mismatched += 1
            print(f"{name}（{desc}）: 原文不匹配（出现 {text.count(old)} 次，应为 {count} 次）", flush=True)
            continue
        open(path, "w", encoding="utf-8").write(text.replace(old, new))
        ok, failed, invariants = build_and_run(component, os.path.join(WORK, "src"))
        if not ok:
            print(f"{name}（{desc}）: 编译或运行失败 {' | '.join(failed)}", flush=True)
            continue
        found = bool(failed)
        caught += found
        detail = " ".join(failed) + (" | 不变量：" + "；".join(invariants) if invariants else "")
        print(f"{name}（{desc}）: {'已发现' if found else '未发现'}；{detail}", flush=True)
    print(f"== 共 {len(mutations)} 个，发现 {caught} 个，原文不匹配 {mismatched} 个", flush=True)
    return 0 if baseline_ok and caught == len(mutations) else 1


if __name__ == "__main__":
    sys.exit(main())
