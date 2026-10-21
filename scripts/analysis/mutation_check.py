#!/usr/bin/env python3
"""错误注入检查：对实现注入典型错误，确认单测能发现，用来检验测试本身的判别力。

在 scripts/docker/run_tests.sh 留下的容器中运行（由 run_mutation_check.sh 调用），只在 /tmp/mutation
中操作源码副本，不改仓库。每个注入把一段原文替换掉，单独编译该组件和它的单测并运行，有用例失败
即为"已发现"。原文必须恰好出现指定次数；代码改动后匹配不到时报告"原文不匹配"，需要同步更新下表。
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

# 组件：(参与编译的 src/domain 源文件, tests/unit 下的单测)
COMPONENTS = {
    "link_monitor": (["link_monitor.cpp"], ["test_link_monitor.cpp"]),
    "pilot_input": (["pilot_input.cpp", "link_monitor.cpp"], ["test_pilot_input.cpp"]),
    "battery_monitor": (["battery_monitor.cpp"], ["test_battery_monitor.cpp"]),
    "airborne_monitor": (["airborne_monitor.cpp"], ["test_airborne_monitor.cpp"]),
    "return_state_machine": (["return_state_machine.cpp"],
                             ["test_return_state_machine.cpp", "test_return_state_machine_invariants.cpp"]),
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
    ('follow_not_frozen', 'return_state_machine', '  // 不持有控制权或里程计延迟时冻结进度（R-18）\n  if (control_ != ControlState::Owned || c.odom_stale) {\n    return;\n  }', '', '跟随不因失去控制权冻结'),
    ('reclaim_without_offset_check', 'return_state_machine', '  if (!yaw_ok || !offset_ok) {', '  if (!yaw_ok) {', '重新接入不检查偏差（B3）'),
    ('preframes_skipped', 'return_state_machine', '      if (pre_frames_ < config_.pre_offboard_setpoint_count) {', '      if (false) {', '不预发直接申请 OFFBOARD'),
    ('pending_never_lands', 'return_state_machine', '  if (c.now - pending_.start_t + kTimeEpsS >= config_.return_pending_max_s) {', '  if (false) {', '补点等待没有上限'),
    ('battery_starts_from_exited', 'return_state_machine', '  const bool battery_low = !from_exited && c.battery.low;', '  const bool battery_low = c.battery.low;', '退出后低电量也启动返航（D1）'),
    ('hover_skipped', 'return_state_machine', '  if (pending_.link_lost) {\n    hover_start_t_ = c.now;', '  if (false) {\n    hover_start_t_ = c.now;', '失控不先悬停（D16）'),
    ('record_in_mission', 'return_state_machine', '  out.follow_active = phase_ == MissionPhase::Follow &&', '  out.record = true;\n  out.follow_active = phase_ == MissionPhase::Follow &&', '任务中继续录制'),
]


def build_and_run(component, src_dir):
    """编译组件和它的单测并运行，返回 (是否编译成功, 失败的用例, 违反的不变量)。"""
    sources, tests = COMPONENTS[component]
    exe = os.path.join(WORK, "test_" + component)
    cmd = (["g++", "-std=c++17", "-O1", "-I" + os.path.join(WORK, "include"), "-I" + GTEST_INC,
            "-I/usr/include/eigen3"]
           + [os.path.join(src_dir, s) for s in sources]
           + [os.path.join(SRC, "tests/unit", t) for t in tests]
           + [os.path.join(WORK, "gtest_main.cpp"), "-L" + GTEST_LIB, "-lgtest", "-lpthread",
              "-Wl,-rpath," + GTEST_LIB, "-o", exe])
    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0:
        return False, [line for line in build.stderr.splitlines() if "error" in line][:2], []
    run = subprocess.run([exe], capture_output=True, text=True, timeout=900)
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
        for name in os.listdir(os.path.join(SRC, "src/domain")):
            shutil.copy(os.path.join(SRC, "src/domain", name), os.path.join(WORK, "src", name))

    print("== 基线（未注入）", flush=True)
    reset_sources()
    for component in sorted({m[1] for m in mutations}):
        ok, failed, _ = build_and_run(component, os.path.join(WORK, "src"))
        print(f"{component}: {'全部通过' if ok and not failed else '基线失败 ' + ' '.join(failed)}", flush=True)

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
            print(f"{name}（{desc}）: 编译失败 {' | '.join(failed)}", flush=True)
            continue
        found = bool(failed)
        caught += found
        detail = " ".join(failed) + (" | 不变量：" + "；".join(invariants) if invariants else "")
        print(f"{name}（{desc}）: {'已发现' if found else '未发现'}；{detail}", flush=True)
    print(f"== 共 {len(mutations)} 个，发现 {caught} 个，原文不匹配 {mismatched} 个", flush=True)
    return 0 if caught == len(mutations) else 1


if __name__ == "__main__":
    sys.exit(main())
