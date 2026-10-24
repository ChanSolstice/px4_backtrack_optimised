#!/usr/bin/env python3
"""按时间线查看调试日志 events.jsonl（重构方案第 9.2 节）。

用法：show_debug_log.py <运行目录或 events.jsonl> [-t state,control,alert] [-m 任务编号]
时间为相对第一条事件的秒数；f、m 为飞行周期和任务编号。
"""
import argparse
import json
import os
import sys


def describe(event):
    """事件正文：名称，转换写成 from -> to，[规则 原因]，补充信息"""
    parts = []
    if event.get("name"):
        parts.append(event["name"])
    if "from" in event or "to" in event:
        parts.append(f'{event.get("from", "")} -> {event.get("to", "")}'.strip())
    rule_reason = " ".join(x for x in (event.get("rule"), event.get("reason")) if x)
    if rule_reason:
        parts.append(f"[{rule_reason}]")
    if event.get("detail"):
        parts.append(event["detail"])
    if "count" in event:
        parts.append(f'count={event["count"]}')
    return "  ".join(parts)


def main():
    parser = argparse.ArgumentParser(description="按时间线查看调试日志 events.jsonl")
    parser.add_argument("path", help="运行目录或 events.jsonl")
    parser.add_argument("-t", "--types", help="只看这些类型，逗号分隔，如 state,control,alert")
    parser.add_argument("-m", "--mission", type=int, help="只看这个任务编号")
    args = parser.parse_args()

    path = os.path.join(args.path, "events.jsonl") if os.path.isdir(args.path) else args.path
    types = set(args.types.split(",")) if args.types else None
    events = []
    with open(path, encoding="utf-8") as f:
        for number, line in enumerate(f, 1):
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                # 节点被强行结束时最后一行可能没写完
                print(f"第 {number} 行不完整，跳过", file=sys.stderr)
    if not events:
        sys.exit("没有事件")

    t0 = events[0]["t"]
    for event in events:
        if event["type"] == "config":
            version = event["version"]
            print(f'config  module={version["module"]} trajectory_core={version["trajectory_core"]}'
                  f'  参数 {len(event["params"])} 个')
            continue
        if types and event["type"] not in types:
            continue
        if args.mission is not None and event.get("mission") != args.mission:
            continue
        print(f'{event["t"] - t0:9.3f}  f{event.get("flight", 0)} m{event.get("mission", 0)}'
              f'  {event["type"]:<12} {describe(event)}')


if __name__ == "__main__":
    main()
