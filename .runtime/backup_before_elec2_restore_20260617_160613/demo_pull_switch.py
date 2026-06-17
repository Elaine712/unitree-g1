#!/usr/bin/env python3
"""Demo: pull a wall-mounted industrial switch using taught arm poses."""

import argparse
import json
import os
import threading
import time

from g1_remote_client import G1RemoteClient


def default_host():
    env = os.environ.get("G1_BACKEND_HOST")
    if env:
        return env
    if os.environ.get("USER") == "unitree" or os.path.exists("/home/unitree/zgx_g1"):
        return "127.0.0.1"
    return "10.231.138.24"


def default_poses_path():
    env = os.environ.get("HONGTU_POSES_PATH")
    if env:
        return env
    candidates = [
        os.path.expanduser("~/Desktop/g1_poses2.json"),
        os.path.join(os.getcwd(), "g1_poses2.json"),
        "/home/unitree/zgx_g1/g1_poses2.json",
        "/home/unitree/g1_poses2.json",
    ]
    existing = [path for path in candidates if os.path.exists(path)]
    return existing[0] if existing else candidates[0]


def pose_paths(value):
    paths = []
    for item in (value or "").replace(",", ":").split(":"):
        item = os.path.expanduser(item.strip())
        if item and item not in paths:
            paths.append(item)
    return paths


def load_pose(paths, name):
    seen = []
    for path in pose_paths(paths):
        if not os.path.exists(path):
            continue
        with open(path, "r", encoding="utf-8") as f:
            poses = json.load(f)
        for pose in poses:
            pose_name = pose.get("name", "")
            seen.append(pose_name)
            if pose_name == name:
                return pose
    raise RuntimeError(f"未找到姿态 {name!r}; 当前有: {', '.join(seen)}")


def load_pose_optional(path, name):
    if not name:
        return None
    try:
        return load_pose(path, name)
    except RuntimeError:
        return None


def arm_from_pose(pose):
    arm = [float(v) for v in pose.get("arm", [])]
    if len(arm) != 14:
        raise RuntimeError(f"姿态 {pose.get('name')} 的 arm 不是 14 维")
    return arm


def send_hand_if_present(client, pose):
    hand_r = pose.get("hand_r")
    hand_l = pose.get("hand_l")
    if hand_r:
        client.hand_angles("r", hand_r)
    if hand_l:
        client.hand_angles("l", hand_l)


class HandKeeper:
    def __init__(self, client, pose, interval=0.05):
        self.client = G1RemoteClient(client.base_url, timeout=client.timeout)
        self.pose = pose
        self.interval = interval
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def _loop(self):
        while not self.stop_event.is_set():
            try:
                send_hand_if_present(self.client, self.pose)
            except Exception as e:
                print(f"[pull] 保持抓握手型失败: {e}")
            time.sleep(self.interval)


def send_pose(client, pose, settle):
    arm = arm_from_pose(pose)
    client.arm_joints(arm)
    send_hand_if_present(client, pose)
    time.sleep(settle)
    return arm


def send_arm_smooth(client, start, end, duration, steps, hold_hand_pose=None):
    steps = max(1, int(steps))
    duration = max(0.0, float(duration))
    for i in range(1, steps + 1):
        a = i / steps
        target = [(1.0 - a) * float(s) + a * float(e) for s, e in zip(start, end)]
        client.arm_joints(target)
        if hold_hand_pose:
            send_hand_if_present(client, hold_hand_pose)
        if duration > 0:
            time.sleep(duration / steps)
    return list(end)


def hold_arm_and_hand(client, arm, hand_pose, seconds, interval=0.08):
    end = time.monotonic() + max(0.0, float(seconds))
    while time.monotonic() < end:
        client.arm_joints(arm)
        send_hand_if_present(client, hand_pose)
        time.sleep(interval)


def max_abs_error(a, b):
    return max([abs(float(x) - float(y)) for x, y in zip(a, b)] + [0.0])


def wait_arm_target(client, target, seconds, tolerance, interval=0.08, hold_hand_pose=None):
    end = time.monotonic() + max(0.0, float(seconds))
    last_err = None
    while time.monotonic() < end:
        client.arm_joints(target)
        if hold_hand_pose:
            send_hand_if_present(client, hold_hand_pose)
        time.sleep(interval)
        try:
            cur = client.arm_current().get("data", {}).get("joints", [])
            if len(cur) >= len(target):
                last_err = max_abs_error(cur[:len(target)], target)
                if last_err <= tolerance:
                    print(f"[pull] 姿态到位 err={last_err:.3f}")
                    return True
        except Exception as e:
            print(f"[pull] 读取臂状态失败，继续保持目标: {e}")
    if last_err is None:
        print("[pull] 姿态保持完成（未读取到误差）")
    else:
        print(f"[pull] 姿态等待超时 err={last_err:.3f}")
    return False


def speak(client, enabled, text):
    if not enabled or not text:
        return
    try:
        client.speak(text)
    except Exception as e:
        print(f"[pull] 语音失败: {e}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=default_host())
    ap.add_argument("--poses", default=default_poses_path())
    ap.add_argument("--approach-name", default="拉闸接近动作")
    ap.add_argument("--grasp-name", default="拉闸抓握动作")
    ap.add_argument("--pull-name", default="拉闸下拉动作")
    ap.add_argument("--safe-name", default="", help="可选：拉闸后先经过的安全脱离动作名；留空则直接回初始")
    ap.add_argument("--init-name", default="初始位置")
    ap.add_argument("--pull-duration", type=float, default=0.7)
    ap.add_argument("--pull-steps", type=int, default=14)
    ap.add_argument("--release-duration", type=float, default=1.0)
    ap.add_argument("--release-steps", type=int, default=20)
    ap.add_argument("--lift-joint-index", type=int, default=7, help="可选脱离补偿关节索引，默认右肩前后")
    ap.add_argument("--lift-delta", type=float, default=0.0, help="可选：拉闸后额外脱离补偿量(rad)，默认 0=不做回弹/下压")
    ap.add_argument("--return-duration", type=float, default=2.0)
    ap.add_argument("--return-steps", type=int, default=30)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--hold-seconds", type=float, default=0.8)
    ap.add_argument("--wait-target", type=float, default=1.0)
    ap.add_argument("--open-after-return-delay", type=float, default=1.2, help="保留参数：当前不额外发送张手指令")
    ap.add_argument("--pre-release-joint-index", type=int, default=7, help="松手前继续下移的关节索引，默认右肩前后")
    ap.add_argument("--pre-release-delta", type=float, default=0.0, help="松手前额外动作量(rad)，默认 0=禁用")
    ap.add_argument("--pre-release-duration", type=float, default=0.35, help="松手前下移动作时间(s)")
    ap.add_argument("--pre-release-steps", type=int, default=8, help="松手前下移动作插值步数")
    ap.add_argument("--target-tolerance", type=float, default=0.08)
    ap.add_argument("--speak", type=int, default=0, help="1=执行开始/完成语音播报")
    ap.add_argument("--start-text", default="到达目的地，检测到电闸未关闭导致漏电，现关闭电闸")
    ap.add_argument("--done-text", default="任务完成，请验收")
    args = ap.parse_args()

    client = G1RemoteClient(f"http://{args.host}:5055", timeout=4.0)
    approach_pose = load_pose(args.poses, args.approach_name)
    grasp_pose = load_pose(args.poses, args.grasp_name)
    pull_pose = load_pose(args.poses, args.pull_name)
    safe_pose = load_pose_optional(args.poses, args.safe_name)
    init_pose = load_pose(args.poses, args.init_name)

    print("[pull] 激活 arm_sdk")
    client.arm_activate()
    current_arm = None
    try:
        speak(client, args.speak, args.start_text)

        print(f"[pull] 接近: {args.approach_name}")
        current_arm = send_pose(client, approach_pose, args.settle)
        wait_arm_target(client, current_arm, args.wait_target, args.target_tolerance)

        print(f"[pull] 抓握/勾住: {args.grasp_name}")
        grasp_arm = send_pose(client, grasp_pose, args.settle)
        wait_arm_target(client, grasp_arm, args.wait_target, args.target_tolerance)
        current_arm = grasp_arm

        print(f"[pull] 拉闸: {args.grasp_name} -> {args.pull_name}")
        pull_arm = arm_from_pose(pull_pose)
        current_arm = send_arm_smooth(client, current_arm, pull_arm, args.pull_duration, args.pull_steps)
        send_hand_if_present(client, pull_pose)
        if args.hold_seconds > 0:
            print(f"[pull] 保持拉闸终点 {args.hold_seconds:.1f}s")
            time.sleep(args.hold_seconds)

        if abs(args.pre_release_delta) > 1e-6:
            print(f"[pull] 松手前执行额外动作 {args.pre_release_delta:.3f}rad")
            pre_release_arm = list(current_arm)
            if args.pre_release_joint_index < 0 or args.pre_release_joint_index >= len(pre_release_arm):
                raise RuntimeError(f"pre-release-joint-index 越界: {args.pre_release_joint_index}")
            pre_release_arm[args.pre_release_joint_index] += args.pre_release_delta
            current_arm = send_arm_smooth(
                client,
                current_arm,
                pre_release_arm,
                args.pre_release_duration,
                args.pre_release_steps,
                hold_hand_pose=pull_pose,
            )

        if safe_pose:
            print(f"[pull] 脱离/抬手安全动作: {args.safe_name}")
            safe_arm = arm_from_pose(safe_pose)
            current_arm = send_arm_smooth(
                client,
                current_arm,
                safe_arm,
                args.release_duration,
                args.release_steps,
                hold_hand_pose=pull_pose,
            )
        elif abs(args.lift_delta) > 1e-6:
            print(f"[pull] 执行可选脱离补偿 {args.lift_delta:.3f}rad")
            lift_arm = list(current_arm)
            if args.lift_joint_index < 0 or args.lift_joint_index >= len(lift_arm):
                raise RuntimeError(f"lift-joint-index 越界: {args.lift_joint_index}")
            lift_arm[args.lift_joint_index] += args.lift_delta
            current_arm = send_arm_smooth(
                client,
                current_arm,
                lift_arm,
                args.release_duration,
                args.release_steps,
                hold_hand_pose=pull_pose,
            )
        else:
            print("[pull] 不执行拉闸后回弹/额外下压，直接回初始")

        print(f"[pull] 回初始动作: {args.init_name}")
        init_arm = arm_from_pose(init_pose)
        current_arm = send_arm_smooth(
            client,
            current_arm,
            init_arm,
            args.return_duration,
            args.return_steps,
            hold_hand_pose=pull_pose,
        )
        hold_arm_and_hand(client, init_arm, pull_pose, args.open_after_return_delay)
        print("[pull] 已回到初始位置，手型由初始位置 JSON 决定")
        send_hand_if_present(client, init_pose)
        time.sleep(args.settle)

        speak(client, args.speak, args.done_text)
    finally:
        print("[pull] 释放 arm_sdk")
        client.arm_release()

    print("[pull] result: done")


if __name__ == "__main__":
    main()
