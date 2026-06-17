#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


SCRIPT_DIR = Path(__file__).resolve().parent
for path in [SCRIPT_DIR, Path("/home/ybbb/g1_dev/mimo/tools")]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from wall_black_line_detector import detect_wall_black_line, draw_overlay


class RosCmdVelClient:
    def __init__(self, topic="/cmd_vel", rate_hz=20.0):
        self.topic = topic
        self.rate_hz = float(rate_hz)
        self.pub = rospy.Publisher(topic, Twist, queue_size=1)
        time.sleep(0.3)

    def _publish(self, vx, vy, omega):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)
        self.pub.publish(msg)

    def SetVelocity(self, vx, vy, omega, duration=1.0):
        end = time.time() + max(0.0, float(duration))
        period = 1.0 / max(1.0, self.rate_hz)
        while time.time() < end and not rospy.is_shutdown():
            self._publish(vx, vy, omega)
            time.sleep(period)
        return 0

    def StopMove(self):
        for _ in range(6):
            self._publish(0.0, 0.0, 0.0)
            time.sleep(0.03)
        return 0


def depth_to_meters(msg):
    if msg.encoding == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        img = arr.reshape(msg.height, msg.step // 2)[:, : msg.width]
        return img.astype(np.float32) * 0.001
    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32)
        return arr.reshape(msg.height, msg.step // 4)[:, : msg.width].copy()
    raise ValueError("unsupported depth encoding: " + msg.encoding)


def ros_image_to_bgr(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError("unsupported color encoding: " + msg.encoding)
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = arr.reshape(msg.height, msg.step)[:, : msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


def init_g1(args):
    if args.motion_mode == "ros_cmd_vel":
        return RosCmdVelClient(args.cmd_vel_topic, args.cmd_vel_rate_hz)

    for p in ["/home/unitree/zgx_g1/unitree_sdk2_python", "/home/unitree/zgx_g1/.runtime/python"]:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    ChannelFactoryInitialize(0, args.motion_net_if)
    client = LocoClient()
    client.SetTimeout(5.0)
    client.Init()
    if args.prepare_loco:
        client.Start()
    return client


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def read_odom(topic, timeout=0.8):
    try:
        msg = rospy.wait_for_message(topic, Odometry, timeout=timeout)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        return {
            "topic": topic,
            "stamp": msg.header.stamp.to_sec(),
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(yaw_from_quat(q)),
            "vx": float(t.linear.x),
            "vy": float(t.linear.y),
            "wz": float(t.angular.z),
        }
    except Exception as e:
        return {"topic": topic, "error": str(e)}


def send_velocity(client, vx, vy, omega, duration, odom_topic):
    before = read_odom(odom_topic)
    move_code = client.SetVelocity(float(vx), float(vy), float(omega), float(duration))
    mid_sleep = min(max(float(duration) * 0.5, 0.15), 0.6)
    time.sleep(mid_sleep)
    during = read_odom(odom_topic)
    time.sleep(max(0.0, float(duration) + 0.20 - mid_sleep))
    stop_code = client.StopMove()
    after = read_odom(odom_topic)
    trace = {
        "motion": "sent",
        "cmd_vx": float(vx),
        "cmd_vy": float(vy),
        "cmd_omega": float(omega),
        "cmd_duration": float(duration),
        "move_code": move_code,
        "stop_code": stop_code,
        "odom_before": before,
        "odom_during": during,
        "odom_after": after,
    }
    if "x" in before and "x" in after:
        trace["odom_delta"] = {
            "dx": after["x"] - before["x"],
            "dy": after["y"] - before["y"],
            "dyaw": angle_delta(after["yaw"], before["yaw"]),
        }
    return trace


def piecewise_step(abs_angle_deg, max_omega, max_duration):
    tiers = [
        (20.0, 0.50, 0.75),
        (12.0, 0.45, 0.65),
        (8.0, 0.38, 0.52),
        (5.0, 0.42, 0.60),
        (3.0, 0.40, 0.55),
        (2.0, 0.38, 0.50),
        (0.0, 0.34, 0.45),
    ]
    for threshold, omega, duration in tiers:
        if abs_angle_deg >= threshold:
            return min(omega, max_omega), min(duration, max_duration)
    return min(0.34, max_omega), min(0.45, max_duration)


def adaptive_step(control_angle, args):
    abs_angle = abs(control_angle)
    if args.step_profile == "piecewise":
        omega, duration = piecewise_step(abs_angle, args.max_omega, args.max_duration)
    else:
        omega = min(args.max_omega, max(args.min_omega, args.kp_omega * abs_angle))
        duration = min(args.max_duration, max(args.min_duration, args.kp_duration * abs_angle))
    return math.copysign(omega, -1.0 if control_angle > 0 else 1.0), duration


def damp_oscillating_yaw_step(control_angle, last_turn_angle, omega, duration, args):
    if last_turn_angle is None:
        return omega, duration, False
    if control_angle * last_turn_angle >= 0:
        return omega, duration, False
    if abs(control_angle) > args.yaw_damping_angle_deg or abs(last_turn_angle) > args.yaw_damping_angle_deg:
        return omega, duration, False
    sign = 1.0 if omega >= 0 else -1.0
    damped_omega = max(args.yaw_damping_min_omega, abs(omega) * args.yaw_damping_omega_scale)
    damped_duration = max(args.yaw_damping_min_duration, duration * args.yaw_damping_duration_scale)
    return sign * min(damped_omega, abs(omega)), min(damped_duration, duration), True


def line_sample_points(result, width, height, step_px=6.0):
    p0 = result.get("hough_segment_p1_uv") or result.get("line_p1_uv")
    p1 = result.get("hough_segment_p2_uv") or result.get("line_p2_uv")
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    length = float(np.linalg.norm(p1 - p0))
    if length < 1.0:
        return np.empty((0, 2), dtype=np.int32)
    count = max(2, int(math.ceil(length / max(1.0, step_px))) + 1)
    ts = np.linspace(0.0, 1.0, count, dtype=np.float32)
    pts = p0[None, :] * (1.0 - ts[:, None]) + p1[None, :] * ts[:, None]
    pts = np.round(pts).astype(np.int32)
    pts[:, 0] = np.clip(pts[:, 0], 0, width - 1)
    pts[:, 1] = np.clip(pts[:, 1], 0, height - 1)
    return pts


def estimate_line_depth(depth_m, result, args):
    height, width = depth_m.shape[:2]
    pts = line_sample_points(result, width, height, step_px=args.depth_sample_step_px)
    if pts.size == 0:
        return None

    radius = max(0, int(args.depth_sample_radius_px))
    values = []
    sample_pixels = 0
    for x, y in pts:
        x0 = max(0, int(x) - radius)
        x1 = min(width, int(x) + radius + 1)
        y0 = max(0, int(y) - radius)
        y1 = min(height, int(y) + radius + 1)
        patch = depth_m[y0:y1, x0:x1].reshape(-1)
        valid = patch[
            np.isfinite(patch)
            & (patch >= args.min_valid_depth_m)
            & (patch <= args.max_valid_depth_m)
        ]
        if valid.size == 0:
            continue
        values.append(float(np.min(valid)))
        sample_pixels += int(valid.size)

    if len(values) < args.min_depth_samples:
        return None
    vals = np.asarray(values, dtype=np.float32)
    return {
        "line_forward_min_distance_m": float(np.min(vals)),
        "line_forward_min_distance_mm": float(np.min(vals) * 1000.0),
        "line_forward_median_distance_m": float(np.median(vals)),
        "line_forward_median_distance_mm": float(np.median(vals) * 1000.0),
        "line_forward_p10_distance_m": float(np.percentile(vals, 10)),
        "line_forward_p10_distance_mm": float(np.percentile(vals, 10) * 1000.0),
        "line_depth_sample_count": int(len(vals)),
        "line_depth_valid_pixel_count": int(sample_pixels),
    }


def forward_step(distance_m, args):
    remaining_m = distance_m - args.target_max_distance_m
    if remaining_m >= 0.20:
        return args.forward_velocity, min(args.forward_duration, args.max_forward_duration)
    if remaining_m >= 0.10:
        return min(args.forward_velocity, args.mid_forward_velocity), min(args.forward_duration, args.mid_forward_duration)
    return min(args.forward_velocity, args.near_forward_velocity), min(args.forward_duration, args.near_forward_duration)


def main():
    parser = argparse.ArgumentParser(description="Align G1 yaw to the visual wall black line.")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--out-dir", default="/home/unitree/g1_dev/yolo11/wall_line_align_attempt")
    parser.add_argument("--max-steps", type=int, default=1)
    parser.add_argument("--deadband-deg", type=float, default=1.5, help="Final yaw deadband after target distance is reached.")
    parser.add_argument("--approach-deadband-deg", type=float, default=2.0, help="Yaw deadband while moving toward the wall line.")
    parser.add_argument("--angle-only", action="store_true", help="Only correct yaw to deadband; ignore distance/forward control.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--step-profile", choices=("linear", "piecewise"), default="linear")
    parser.add_argument("--pid-step", action="store_true")
    parser.add_argument("--omega", type=float, default=0.34)
    parser.add_argument("--duration", type=float, default=0.45)
    parser.add_argument("--min-omega", type=float, default=0.45)
    parser.add_argument("--max-omega", type=float, default=0.56)
    parser.add_argument("--kp-omega", type=float, default=0.055)
    parser.add_argument("--min-duration", type=float, default=0.50)
    parser.add_argument("--max-duration", type=float, default=0.60)
    parser.add_argument("--kp-duration", type=float, default=0.065)
    parser.add_argument("--yaw-damping-angle-deg", type=float, default=5.0)
    parser.add_argument("--yaw-damping-omega-scale", type=float, default=0.88)
    parser.add_argument("--yaw-damping-duration-scale", type=float, default=0.92)
    parser.add_argument("--yaw-damping-min-omega", type=float, default=0.45)
    parser.add_argument("--yaw-damping-min-duration", type=float, default=0.50)
    parser.add_argument("--roi-y1-ratio", type=float, default=0.68)
    parser.add_argument("--roi-y2-ratio", type=float, default=0.98)
    parser.add_argument("--dark-threshold", type=int, default=85)
    parser.add_argument("--target-min-distance-m", type=float, default=1.20)
    parser.add_argument("--target-max-distance-m", type=float, default=1.30)
    parser.add_argument("--min-valid-depth-m", type=float, default=0.20)
    parser.add_argument("--max-valid-depth-m", type=float, default=4.0)
    parser.add_argument("--depth-sample-radius-px", type=int, default=2)
    parser.add_argument("--depth-sample-step-px", type=float, default=6.0)
    parser.add_argument("--min-depth-samples", type=int, default=24)
    parser.add_argument("--forward-velocity", type=float, default=0.40)
    parser.add_argument("--forward-duration", type=float, default=0.50)
    parser.add_argument("--mid-forward-velocity", type=float, default=0.45)
    parser.add_argument("--mid-forward-duration", type=float, default=0.50)
    parser.add_argument("--near-forward-velocity", type=float, default=0.50)
    parser.add_argument("--near-forward-duration", type=float, default=0.50)
    parser.add_argument("--max-forward-duration", type=float, default=0.35)
    parser.add_argument("--max-no-line-frames", type=int, default=4)
    parser.add_argument("--max-no-depth-frames", type=int, default=4)
    parser.add_argument("--motion-mode", choices=("ros_cmd_vel", "sdk"), default=os.environ.get("HONGTU_ALIGN_MOTION_MODE", "ros_cmd_vel"))
    parser.add_argument("--cmd-vel-topic", default=os.environ.get("HONGTU_ALIGN_CMD_VEL_TOPIC", "/cmd_vel"))
    parser.add_argument("--cmd-vel-rate-hz", type=float, default=float(os.environ.get("HONGTU_ALIGN_CMD_VEL_RATE_HZ", "20")))
    parser.add_argument("--motion-net-if", default=os.environ.get("HONGTU_ALIGN_MOTION_NET_IF", "eth0"))
    parser.add_argument("--motion-odom-topic", default=os.environ.get("HONGTU_ALIGN_ODOM_TOPIC", "/g1/LowFre_Odom"))
    parser.set_defaults(prepare_loco=True)
    parser.add_argument("--prepare-loco", dest="prepare_loco", action="store_true")
    parser.add_argument("--no-prepare-loco", dest="prepare_loco", action="store_false")
    args = parser.parse_args()
    if args.target_min_distance_m > args.target_max_distance_m:
        raise SystemExit("--target-min-distance-m must be <= --target-max-distance-m")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rospy.init_node("g1_wall_black_line_align_once", anonymous=True)
    client = None if args.dry_run else init_g1(args)
    records = []
    last_turn_angle = None
    no_line_frames = 0
    no_depth_frames = 0

    for step in range(args.max_steps):
        msg = rospy.wait_for_message(args.image_topic, Image, timeout=10)
        depth_msg = rospy.wait_for_message(args.depth_topic, Image, timeout=10)
        frame = ros_image_to_bgr(msg)
        depth_m = depth_to_meters(depth_msg)
        result, debug = detect_wall_black_line(
            frame,
            roi_y1_ratio=args.roi_y1_ratio,
            roi_y2_ratio=args.roi_y2_ratio,
            dark_threshold=args.dark_threshold,
        )

        if result is None:
            action = f"step={step} no_line"
            overlay = draw_overlay(frame, None, debug)
            cv2.imwrite(str(out_dir / f"step_{step:02d}_overlay.jpg"), overlay)
            rec = {"step": step, "status": "no_line"}
            records.append(rec)
            print(json.dumps(rec, ensure_ascii=False), flush=True)
            no_line_frames += 1
            if args.dry_run or no_line_frames >= args.max_no_line_frames:
                break
            time.sleep(0.25)
            continue
        no_line_frames = 0

        angle = float(result["angle_deg"])
        line_depth = estimate_line_depth(depth_m, result, args)
        if args.angle_only:
            active_deadband_deg = args.deadband_deg
            distance_in_range = False
            too_close = False
            vx_cmd = 0.0
            if abs(angle) <= args.deadband_deg:
                command = "aligned"
                omega_cmd = 0.0
                duration_cmd = 0.0
            elif angle < 0:
                command = "left"
                if args.pid_step:
                    omega_cmd, duration_cmd = adaptive_step(angle, args)
                else:
                    omega_cmd, duration_cmd = abs(args.omega), args.duration
            else:
                command = "right"
                if args.pid_step:
                    omega_cmd, duration_cmd = adaptive_step(angle, args)
                else:
                    omega_cmd, duration_cmd = -abs(args.omega), args.duration
        elif line_depth is None:
            command = "no_depth"
            vx_cmd = 0.0
            omega_cmd = 0.0
            duration_cmd = 0.0
            no_depth_frames += 1
        else:
            no_depth_frames = 0
            line_distance_m = line_depth["line_forward_min_distance_m"]
            distance_in_range = args.target_min_distance_m <= line_distance_m <= args.target_max_distance_m
            too_close = line_distance_m < args.target_min_distance_m
            active_deadband_deg = args.deadband_deg if distance_in_range or too_close else args.approach_deadband_deg

            if abs(angle) <= args.deadband_deg and distance_in_range:
                command = "aligned"
                vx_cmd = 0.0
                omega_cmd = 0.0
                duration_cmd = 0.0
            elif too_close:
                command = "too_close" if abs(angle) <= args.deadband_deg else ("left" if angle < 0 else "right")
                vx_cmd = 0.0
                if command == "too_close":
                    omega_cmd = 0.0
                    duration_cmd = 0.0
                elif args.pid_step:
                    omega_cmd, duration_cmd = adaptive_step(angle, args)
                else:
                    omega_cmd = abs(args.omega) if angle < 0 else -abs(args.omega)
                    duration_cmd = args.duration
            elif abs(angle) <= active_deadband_deg:
                command = "forward"
                vx_cmd, duration_cmd = forward_step(line_distance_m, args)
                omega_cmd = 0.0
            elif angle < 0:
                command = "left"
                vx_cmd = 0.0
                if args.pid_step:
                    omega_cmd, duration_cmd = adaptive_step(angle, args)
                else:
                    omega_cmd, duration_cmd = abs(args.omega), args.duration
            else:
                command = "right"
                vx_cmd = 0.0
                if args.pid_step:
                    omega_cmd, duration_cmd = adaptive_step(angle, args)
                else:
                    omega_cmd, duration_cmd = -abs(args.omega), args.duration

        if args.angle_only:
            active_deadband_deg = args.deadband_deg
            distance_in_range = False
            too_close = False
        elif line_depth is None:
            active_deadband_deg = args.approach_deadband_deg
            distance_in_range = False
            too_close = False
        else:
            line_distance_m = line_depth["line_forward_min_distance_m"]
            distance_in_range = args.target_min_distance_m <= line_distance_m <= args.target_max_distance_m
            too_close = line_distance_m < args.target_min_distance_m

        yaw_damped = False
        if command in ("left", "right"):
            omega_cmd, duration_cmd, yaw_damped = damp_oscillating_yaw_step(
                angle,
                last_turn_angle,
                omega_cmd,
                duration_cmd,
                args,
            )

        action = (
            f"step={step} wall_line={angle:+.2f}deg command={command} "
            f"vx={vx_cmd:+.2f} omega={omega_cmd:+.3f} duration={duration_cmd:.2f}s "
            f"dist={(line_depth['line_forward_min_distance_mm'] if line_depth else -1):.0f}mm"
        )
        overlay = draw_overlay(frame, result, debug)
        cv2.putText(overlay, action, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(overlay, action, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(out_dir / f"step_{step:02d}_overlay.jpg"), overlay)

        rec = {
            "step": step,
            "status": "ok",
            "image_stamp": msg.header.stamp.to_sec(),
            "angle_deg": angle,
            "deadband_deg": args.deadband_deg,
            "command": command,
            "vx_cmd": vx_cmd,
            "omega_cmd": omega_cmd,
            "duration": duration_cmd,
            "yaw_damped": yaw_damped,
            "active_deadband_deg": active_deadband_deg,
            "approach_deadband_deg": args.approach_deadband_deg,
            "target_min_distance_m": args.target_min_distance_m,
            "target_max_distance_m": args.target_max_distance_m,
            "distance_in_range": distance_in_range,
            "too_close": too_close,
            "angle_only": args.angle_only,
            "line_depth": line_depth,
            "detector": result,
        }
        records.append(rec)
        print(json.dumps(rec, ensure_ascii=False), flush=True)

        if command in ("aligned", "too_close"):
            if client is not None:
                client.StopMove()
            break
        if command == "no_depth" and no_depth_frames >= args.max_no_depth_frames:
            if client is not None:
                client.StopMove()
            break
        if args.dry_run:
            break

        motion_trace = send_velocity(client, vx_cmd, 0.0, omega_cmd, duration_cmd, args.motion_odom_topic)
        motion_trace["motion_mode"] = args.motion_mode
        motion_trace["cmd_vel_topic"] = args.cmd_vel_topic if args.motion_mode == "ros_cmd_vel" else None
        rec["motion_trace"] = motion_trace
        print(json.dumps({"step": step, "motion_trace": motion_trace}, ensure_ascii=False), flush=True)
        time.sleep(0.8)
        if command in ("left", "right"):
            last_turn_angle = angle

    with open(out_dir / "records.json", "w") as f:
        json.dump(records, f, indent=2)
    print("records:", out_dir / "records.json", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    # Unitree SDK / DDS native teardown can segfault after a successful run.
    # All commands, StopMove calls, overlays, and records are complete here.
    os._exit(0)


if __name__ == "__main__":
    main()
