#!/usr/bin/env python3
"""Demo: press a switch with one arm joint and stop on Inspire hand force."""

import argparse
import json
import os
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
        os.path.expanduser("~/Desktop/g1_poses.json"),
        os.path.join(os.getcwd(), "g1_poses.json"),
        "/home/unitree/zgx_g1/g1_poses.json",
        "/home/unitree/g1_poses.json",
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


def load_pose(path, name):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        poses = json.load(f)
    for pose in poses:
        if pose.get("name") == name:
            return pose
    names = ", ".join(p.get("name", "") for p in poses)
    raise RuntimeError(f"未找到姿态 {name!r}; 当前有: {names}")


def load_pose_optional(path, name):
    if not name:
        return None
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        poses = json.load(f)
    for pose in poses:
        if pose.get("name") == name:
            return pose
    return None


def hand_force(client, side):
    data = client.status().get("data", {})
    state = data.get("hand_state", {}).get(side, {})
    return [int(v) for v in state.get("force", [])]


def max_force_delta(zero, cur):
    return max([max(0, int(c) - int(z)) for z, c in zip(zero, cur)] + [0])


def send_pose(client, pose, side, settle):
    arm = [float(v) for v in pose.get("arm", [])]
    if len(arm) != 14:
        raise RuntimeError(f"姿态 {pose.get('name')} 的 arm 不是 14 维")
    client.arm_joints(arm)
    hand = pose.get("hand_r")
    if hand and side == "r":
        client.hand_angles("r", hand)
    time.sleep(settle)
    return arm


def send_arm_smooth(client, start, end, duration, steps):
    steps = max(1, int(steps))
    duration = max(0.0, float(duration))
    for i in range(1, steps + 1):
        a = i / steps
        target = [(1.0 - a) * float(s) + a * float(e) for s, e in zip(start, end)]
        client.arm_joints(target)
        if duration > 0:
            time.sleep(duration / steps)
    return list(end)


def hold_arm_pose(client, pose, seconds, interval=0.08):
    end = time.monotonic() + max(0.0, float(seconds))
    while time.monotonic() < end:
        client.arm_joints(pose)
        time.sleep(interval)


def max_abs_error(a, b):
    return max([abs(float(x) - float(y)) for x, y in zip(a, b)] + [0.0])


def wait_arm_target(client, pose, seconds, tolerance, interval=0.08):
    end = time.monotonic() + max(0.0, float(seconds))
    last_err = None
    while time.monotonic() < end:
        client.arm_joints(pose)
        time.sleep(interval)
        try:
            cur = client.arm_current().get("data", {}).get("joints", [])
            if len(cur) >= len(pose):
                last_err = max_abs_error(cur[:len(pose)], pose)
                if last_err <= tolerance:
                    print(f"[demo] 收手过渡态已到位 err={last_err:.3f}")
                    return True
        except Exception as e:
            print(f"[demo] 读取臂状态失败，继续保持过渡态: {e}")
    if last_err is None:
        print("[demo] 收手过渡态保持完成（未读取到关节误差）")
    else:
        print(f"[demo] 收手过渡态等待超时 err={last_err:.3f}")
    return False


def parse_coupling(text):
    pairs = []
    for item in (text or "").split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise RuntimeError(f"press-coupling 格式错误: {item}")
        idx, scale = item.split(":", 1)
        pairs.append((int(idx), float(scale)))
    return pairs


def make_press_target(base, main_index, direction, moved, coupling):
    target = list(base)
    delta = direction * moved
    target[main_index] = base[main_index] + delta
    for idx, scale in coupling:
        if idx < 0 or idx >= len(target):
            raise RuntimeError(f"补偿关节索引越界: {idx}")
        target[idx] = base[idx] + delta * scale
    return target


def make_taught_press_target(base, down, max_delta, moved):
    if not down:
        return None
    ratio = max(0.0, min(1.0, moved / max(max_delta, 1e-6)))
    return [(1.0 - ratio) * float(s) + ratio * float(e) for s, e in zip(base, down)]


def send_taught_press(client, base, down, duration, steps, zero, side):
    steps = max(1, int(steps))
    duration = max(0.0, float(duration))
    last = list(base)
    for i in range(1, steps + 1):
        a = i / steps
        target = [(1.0 - a) * float(s) + a * float(e) for s, e in zip(base, down)]
        client.arm_joints(target)
        last = list(target)
        if duration > 0:
            time.sleep(duration / steps)
        cur = hand_force(client, side)
        delta = max_force_delta(zero, cur)
        print(f"[demo] taught_press force_delta={delta} progress={a:.2f}")
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=default_host())
    ap.add_argument("--poses", default=default_poses_path())
    ap.add_argument("--approach-name", default="收手过渡态", help="可选：执行按压预设前先经过的过渡姿态名")
    ap.add_argument("--press-name", default="按压平举预设动作")
    ap.add_argument("--down-name", default="按下动作", help="可选：示教的按下终点姿态名；存在时优先用示教轨迹下压")
    ap.add_argument("--post-press-lift-name", default="按压后抬起动作", help="可选：按压完成后先到达的安全抬起姿态名")
    ap.add_argument("--transition-name", default="收手过渡态", help="可选：回初始前先经过的过渡姿态名")
    ap.add_argument("--init-name", default="初始位置")
    ap.add_argument("--side", default="r", choices=["l", "r"])
    ap.add_argument("--joint-index", type=int, default=7, help="右肩前后在 14 维 arm 中是 7")
    ap.add_argument("--press-coupling", default="10:-0.35,11:0.25", help="多关节按压补偿，如 10:-0.35,11:0.25；留空则只动主关节")
    ap.add_argument("--direction", type=float, default=1.0, help="下压方向；若方向反了改成 -1")
    ap.add_argument("--step", type=float, default=0.004, help="每步关节增量(rad)")
    ap.add_argument("--max-delta", type=float, default=0.40, help="最大下压关节量(rad)")
    ap.add_argument("--rebound", type=float, default=0.05, help="触发后反向回弹量(rad)")
    ap.add_argument("--force-threshold", type=int, default=120)
    ap.add_argument("--post-contact-delta", type=float, default=0.25, help="达到压力阈值后继续下压的关节量(rad)")
    ap.add_argument("--post-contact-dt", type=float, default=0.12, help="接触后继续下压每步间隔(s)")
    ap.add_argument("--taught-press-duration", type=float, default=0.35, help="示教按下动作执行时间(s)，越小越快")
    ap.add_argument("--taught-press-steps", type=int, default=8, help="示教按下动作插值步数")
    ap.add_argument("--settle", type=float, default=1.2)
    ap.add_argument("--dt", type=float, default=0.10, help="每步下压间隔(s)")
    ap.add_argument("--hold-seconds", type=float, default=0.0, help="到达最大下压/触发压力后停留观察(s)")
    ap.add_argument("--success-lift", type=float, default=0.12, help="按压成功后先抬高主关节再收手(rad)")
    ap.add_argument("--success-shoulder-back", type=float, default=0.08, help="按压成功后肩前后关节额外后撤量(rad)，方向不对可设为负数")
    ap.add_argument("--success-lift-duration", type=float, default=0.8, help="按压成功后抬高手臂时间(s)")
    ap.add_argument("--success-lift-steps", type=int, default=16, help="按压成功后抬高手臂步数")
    ap.add_argument("--post-press-return-duration", type=float, default=0.8, help="按压后沿示教轨迹反向抬回按压平举的时间(s)")
    ap.add_argument("--post-press-return-steps", type=int, default=16, help="按压后沿示教轨迹反向抬回按压平举的步数")
    ap.add_argument("--skip-transition-after-lift", type=int, default=1, help="使用按压后抬起动作后跳过收手过渡态，1=跳过")
    ap.add_argument("--safe-lift-duration", type=float, default=1.2, help="先收肩到安全位的时间(s)")
    ap.add_argument("--safe-lift-steps", type=int, default=20, help="先收肩到安全位的步数")
    ap.add_argument("--approach-lift-ratio", type=float, default=0.25, help="接近阶段先抬肩比例，0.25 表示只抬到过渡态到按压态差值的 25%")
    ap.add_argument("--approach-duration", type=float, default=1.5, help="过渡态到按压姿态的分段展开时间(s)")
    ap.add_argument("--approach-steps", type=int, default=24, help="过渡态到按压姿态的分段展开步数")
    ap.add_argument("--transition-hold", type=float, default=1.5, help="到达收手过渡态后保持/等待到位时间(s)")
    ap.add_argument("--transition-tolerance", type=float, default=0.08, help="收手过渡态到位容差(rad)")
    ap.add_argument("--return-duration", type=float, default=2.0, help="回初始动作插值时间(s)")
    ap.add_argument("--return-steps", type=int, default=30, help="回初始动作插值步数")
    args = ap.parse_args()
    coupling = parse_coupling(args.press_coupling)

    client = G1RemoteClient(f"http://{args.host}:5055", timeout=4.0)
    approach_pose = load_pose(args.poses, args.approach_name) if args.approach_name else None
    press_pose = load_pose(args.poses, args.press_name)
    down_pose = load_pose_optional(args.poses, args.down_name)
    post_press_lift_pose = load_pose_optional(args.poses, args.post_press_lift_name)
    transition_pose = load_pose(args.poses, args.transition_name) if args.transition_name else None
    init_pose = load_pose(args.poses, args.init_name)

    print("[demo] 激活 arm_sdk")
    client.arm_activate()
    pressed = False
    last_target = None
    current_arm = None
    try:
        approach_arm = None
        if approach_pose:
            print(f"[demo] 先经过接近过渡动作: {args.approach_name}")
            approach_arm = send_pose(client, approach_pose, args.side, args.settle)

        print(f"[demo] 执行预设: {args.press_name}")
        if approach_arm is not None:
            press_arm = [float(v) for v in press_pose.get("arm", [])]
            if len(press_arm) != 14:
                raise RuntimeError(f"姿态 {args.press_name} 的 arm 不是 14 维")

            print("[demo] 接近阶段：先轻微抬肩")
            lift_arm = list(approach_arm)
            ratio = max(0.0, min(1.0, args.approach_lift_ratio))
            lift_arm[args.joint_index] = (
                approach_arm[args.joint_index]
                + (press_arm[args.joint_index] - approach_arm[args.joint_index]) * ratio
            )
            current_arm = send_arm_smooth(
                client, approach_arm, lift_arm, args.safe_lift_duration, args.safe_lift_steps
            )

            print("[demo] 接近阶段：保持高度后展开到按压姿态")
            base = send_arm_smooth(
                client, current_arm, press_arm, args.approach_duration, args.approach_steps
            )
            hand = press_pose.get("hand_r")
            if hand and args.side == "r":
                client.hand_angles("r", hand)
            time.sleep(args.settle)
        else:
            base = send_pose(client, press_pose, args.side, args.settle)
        current_arm = list(base)
        zero = hand_force(client, args.side)
        if len(zero) < 6:
            raise RuntimeError(f"没有压感数据: {zero}")
        print("[demo] force zero:", zero)
        if coupling:
            print("[demo] 多关节按压补偿:", coupling)
        else:
            print("[demo] 单关节按压")
        down_arm = None
        if down_pose:
            down_arm = [float(v) for v in down_pose.get("arm", [])]
            if len(down_arm) != 14:
                raise RuntimeError(f"姿态 {args.down_name} 的 arm 不是 14 维")
            print(f"[demo] 使用示教下压终点: {args.down_name}")
        else:
            print(f"[demo] 未找到示教下压终点: {args.down_name}，使用多关节补偿下压")

        final_moved = 0.0
        if down_arm:
            print(f"[demo] 快速执行示教按下动作: {args.taught_press_duration:.2f}s")
            last_target = send_taught_press(
                client, base, down_arm, args.taught_press_duration, args.taught_press_steps, zero, args.side
            )
            current_arm = list(last_target)
            pressed = True
        else:
            moved = 0.0
            while moved < args.max_delta:
                cur = hand_force(client, args.side)
                delta = max_force_delta(zero, cur)
                print(f"[demo] force_delta={delta} moved={moved:.3f}")
                if delta >= args.force_threshold:
                    pressed = True
                    break

                moved += args.step
                final_moved = moved
                target = make_press_target(base, args.joint_index, args.direction, moved, coupling)
                last_target = target
                current_arm = list(target)
                client.arm_joints(target)
                time.sleep(args.dt)

            if pressed and last_target and args.post_contact_delta > 0:
                print(f"[demo] 压力达阈值，继续慢压 {args.post_contact_delta:.3f}rad 以触发按钮")
                extra_moved = 0.0
                while extra_moved < args.post_contact_delta and moved + extra_moved < args.max_delta:
                    extra_moved += args.step
                    final_moved = moved + extra_moved
                    target = make_press_target(
                        base, args.joint_index, args.direction, moved + extra_moved, coupling
                    )
                    last_target = target
                    current_arm = list(target)
                    client.arm_joints(target)
                    time.sleep(args.post_contact_dt)
                    cur = hand_force(client, args.side)
                    delta = max_force_delta(zero, cur)
                    print(
                        f"[demo] post_contact force_delta={delta} "
                        f"total_moved={moved + extra_moved:.3f}"
                    )
                if moved + extra_moved >= args.max_delta:
                    print(f"[demo] 已到最大下压行程 max_delta={args.max_delta:.3f}")

        if args.hold_seconds > 0:
            print(f"[demo] 保持当前位置 {args.hold_seconds:.1f}s，便于观察下压位置")
            time.sleep(args.hold_seconds)

        if pressed and last_target and down_arm:
            if post_press_lift_pose:
                print(f"[demo] 示教按下完成，先到安全抬起姿态: {args.post_press_lift_name}")
                lift_arm = [float(v) for v in post_press_lift_pose.get("arm", [])]
                if len(lift_arm) != 14:
                    raise RuntimeError(f"姿态 {args.post_press_lift_name} 的 arm 不是 14 维")
                current_arm = send_arm_smooth(
                    client,
                    last_target,
                    lift_arm,
                    args.post_press_return_duration,
                    args.post_press_return_steps,
                )
                wait_arm_target(client, lift_arm, args.transition_hold, args.transition_tolerance)
            else:
                print("[demo] 未找到安全抬起姿态，沿按压轨迹反向抬回按压平举")
                current_arm = send_arm_smooth(
                    client,
                    last_target,
                    base,
                    args.post_press_return_duration,
                    args.post_press_return_steps,
                )
        elif pressed and last_target:
            print("[demo] 压感到阈值，回弹")
            rebound_moved = max(0.0, final_moved - args.rebound)
            rebound = make_taught_press_target(base, down_arm, args.max_delta, rebound_moved)
            if rebound is None:
                rebound = make_press_target(
                    base, args.joint_index, args.direction, rebound_moved, coupling
                )
            client.arm_joints(rebound)
            current_arm = list(rebound)
            time.sleep(0.5)
        elif not pressed:
            print("[demo] 未达到压力阈值，按超时/最大行程处理")

        if current_arm is None:
            current_arm = list(base)

        skip_transition = bool(args.skip_transition_after_lift and pressed and last_target and down_arm and post_press_lift_pose)
        if skip_transition:
            print("[demo] 已完成按压后抬起，跳过收手过渡态")
        elif transition_pose:
            print(f"[demo] 平滑经过过渡动作: {args.transition_name}")
            transition_arm = [float(v) for v in transition_pose.get("arm", [])]
            if len(transition_arm) != 14:
                raise RuntimeError(f"姿态 {args.transition_name} 的 arm 不是 14 维")
            current_arm = send_arm_smooth(
                client, current_arm, transition_arm, args.return_duration, args.return_steps
            )
            if args.transition_hold > 0:
                print(f"[demo] 等待收手过渡态到位 {args.transition_hold:.1f}s")
                wait_arm_target(client, transition_arm, args.transition_hold, args.transition_tolerance)
        else:
            print("[demo] 先收肩到安全位")
            safe_arm = list(current_arm)
            safe_arm[args.joint_index] = base[args.joint_index]
            current_arm = send_arm_smooth(
                client, current_arm, safe_arm, args.safe_lift_duration, args.safe_lift_steps
            )

        print(f"[demo] 平滑回到初始动作: {args.init_name}")
        init_arm = [float(v) for v in init_pose.get("arm", [])]
        if len(init_arm) != 14:
            raise RuntimeError(f"姿态 {args.init_name} 的 arm 不是 14 维")
        send_arm_smooth(client, current_arm, init_arm, args.return_duration, args.return_steps)
        hand = init_pose.get("hand_r")
        if hand and args.side == "r":
            client.hand_angles("r", hand)
        time.sleep(args.settle)
    finally:
        print("[demo] 释放 arm_sdk")
        client.arm_release()

    print("[demo] result:", "pressed" if pressed else "not_pressed")


if __name__ == "__main__":
    main()
