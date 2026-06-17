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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


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


def load_detections(msg, class_name, class_id, min_conf):
    payload = json.loads(msg.data)
    dets = []
    for det in payload.get("detections", []):
        det_name = det.get("class_name")
        if det_name is not None:
            if det_name != class_name:
                continue
        elif int(det.get("class_id", -1)) != class_id:
            continue
        conf = float(det.get("confidence", 0.0))
        xyxy = det.get("xyxy", [])
        if conf < min_conf or len(xyxy) != 4:
            continue
        x1, y1, x2, y2 = [float(v) for v in xyxy]
        if x2 <= x1 or y2 <= y1:
            continue
        dets.append(
            {
                "class_id": int(det.get("class_id", class_id)),
                "class_name": det_name or class_name,
                "confidence": conf,
                "xyxy": [x1, y1, x2, y2],
                "center_uv": [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
                "area": (x2 - x1) * (y2 - y1),
            }
        )
    dets.sort(key=lambda d: (d["confidence"], d["area"]), reverse=True)
    return payload, dets


def choose_two_dianzha(dets):
    if len(dets) < 2:
        return None
    chosen = dets[:2]
    chosen.sort(key=lambda d: d["center_uv"][0])
    return chosen


def adaptive_lateral_step(abs_error_px, args):
    vy = min(args.max_vy, max(args.min_vy, args.kp_vy * abs_error_px))
    duration = min(args.max_duration, max(args.min_duration, args.kp_duration * abs_error_px))
    return vy, duration


def draw_overlay(frame, chosen, midpoint, error_px, command, vy_cmd, duration_cmd, deadband_px, target_x, target_offset_px):
    overlay = frame.copy()
    height, width = overlay.shape[:2]
    center_x = width * 0.5
    cv2.line(overlay, (int(round(center_x)), 0), (int(round(center_x)), height - 1), (255, 255, 255), 2)
    cv2.line(overlay, (int(round(target_x)), 0), (int(round(target_x)), height - 1), (255, 120, 0), 2)
    cv2.line(overlay, (int(round(target_x - deadband_px)), 0), (int(round(target_x - deadband_px)), height - 1), (80, 80, 80), 1)
    cv2.line(overlay, (int(round(target_x + deadband_px)), 0), (int(round(target_x + deadband_px)), height - 1), (80, 80, 80), 1)

    if chosen is not None:
        centers = []
        for det in chosen:
            x1, y1, x2, y2 = [int(round(v)) for v in det["xyxy"]]
            cx, cy = [int(round(v)) for v in det["center_uv"]]
            centers.append((cx, cy))
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.circle(overlay, (cx, cy), 5, (0, 255, 255), -1)
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(overlay, label, (x1 + 4, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(overlay, label, (x1 + 4, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 1, cv2.LINE_AA)
        if len(centers) == 2:
            cv2.line(overlay, centers[0], centers[1], (0, 255, 255), 2)
            mx, my = [int(round(v)) for v in midpoint]
            cv2.circle(overlay, (mx, my), 7, (0, 0, 255), -1)

    if error_px is None:
        text = "dianzha center: NEED TWO BOXES"
    else:
        text = (
            f"target_offset={target_offset_px:+.0f}px mid_x_error={error_px:+.1f}px command={command} "
            f"vy={vy_cmd:+.3f} duration={duration_cmd:.2f}s db={deadband_px:.0f}px"
        )
    cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    return overlay


def main():
    parser = argparse.ArgumentParser(description="Laterally align G1 to the midpoint of two dianzha1 YOLO boxes.")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--detections-topic", default="/yolo11/detections")
    parser.add_argument("--out-dir", default="/home/unitree/g1_dev/yolo11/dianzha_center_lateral_align_attempt")
    parser.add_argument("--class-name", default="dianzha1")
    parser.add_argument("--class-id", type=int, default=0)
    parser.add_argument("--min-conf", type=float, default=0.30)
    parser.add_argument("--max-steps", type=int, default=1)
    parser.add_argument("--deadband-px", type=float, default=20.0)
    parser.add_argument("--target-offset-px", type=float, default=-30.0, help="Target midpoint offset from image center; negative means left.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--invert-vy", action="store_true", help="Flip lateral direction if G1 moves opposite to image correction.")
    parser.add_argument("--min-vy", type=float, default=0.16)
    parser.add_argument("--max-vy", type=float, default=0.30)
    parser.add_argument("--kp-vy", type=float, default=0.0035)
    parser.add_argument("--min-duration", type=float, default=0.18)
    parser.add_argument("--max-duration", type=float, default=0.35)
    parser.add_argument("--kp-duration", type=float, default=0.004)
    parser.add_argument("--max-no-target-frames", type=int, default=4)
    parser.add_argument("--motion-net-if", default=os.environ.get("HONGTU_ALIGN_MOTION_NET_IF", "eth0"))
    parser.add_argument("--motion-odom-topic", default=os.environ.get("HONGTU_ALIGN_ODOM_TOPIC", "/slam_odom"))
    parser.set_defaults(prepare_loco=True)
    parser.add_argument("--prepare-loco", dest="prepare_loco", action="store_true")
    parser.add_argument("--no-prepare-loco", dest="prepare_loco", action="store_false")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rospy.init_node("g1_dianzha_center_lateral_align_once", anonymous=True)
    client = None if args.dry_run else init_g1(args)
    records = []
    no_target_frames = 0

    for step in range(args.max_steps):
        image_msg = rospy.wait_for_message(args.image_topic, Image, timeout=10)
        det_msg = rospy.wait_for_message(args.detections_topic, String, timeout=10)
        frame = ros_image_to_bgr(image_msg)
        height, width = frame.shape[:2]
        payload, dets = load_detections(det_msg, args.class_name, args.class_id, args.min_conf)
        chosen = choose_two_dianzha(dets)
        target_x = width * 0.5 + args.target_offset_px

        if chosen is None:
            overlay = draw_overlay(frame, None, None, None, "no_target", 0.0, 0.0, args.deadband_px, target_x, args.target_offset_px)
            cv2.imwrite(str(out_dir / f"step_{step:02d}_overlay.jpg"), overlay)
            rec = {
                "step": step,
                "status": "no_two_dianzha",
                "detection_count": len(dets),
                "yolo_seq": payload.get("seq"),
            }
            records.append(rec)
            print(json.dumps(rec, ensure_ascii=False), flush=True)
            no_target_frames += 1
            if args.dry_run or no_target_frames >= args.max_no_target_frames:
                break
            time.sleep(0.25)
            continue
        no_target_frames = 0

        midpoint = [
            0.5 * (chosen[0]["center_uv"][0] + chosen[1]["center_uv"][0]),
            0.5 * (chosen[0]["center_uv"][1] + chosen[1]["center_uv"][1]),
        ]
        image_center_x = width * 0.5
        error_px = float(midpoint[0] - target_x)

        if abs(error_px) <= args.deadband_px:
            command = "aligned"
            vy_cmd = 0.0
            duration_cmd = 0.0
        else:
            speed, duration_cmd = adaptive_lateral_step(abs(error_px), args)
            # If the dianzha midpoint is left of image center, move G1 left so
            # the target shifts right in the image. Unitree torso Y is left.
            vy_cmd = speed if error_px < 0 else -speed
            if args.invert_vy:
                vy_cmd = -vy_cmd
            command = "left" if vy_cmd > 0 else "right"

        overlay = draw_overlay(frame, chosen, midpoint, error_px, command, vy_cmd, duration_cmd, args.deadband_px, target_x, args.target_offset_px)
        cv2.imwrite(str(out_dir / f"step_{step:02d}_overlay.jpg"), overlay)

        rec = {
            "step": step,
            "status": "ok",
            "image_stamp": image_msg.header.stamp.to_sec(),
            "yolo_stamp": payload.get("stamp"),
            "yolo_seq": payload.get("seq"),
            "image_width": width,
            "image_center_x": image_center_x,
            "target_x": target_x,
            "target_offset_px": args.target_offset_px,
            "midpoint_uv": midpoint,
            "midpoint_error_px": error_px,
            "deadband_px": args.deadband_px,
            "command": command,
            "vx_cmd": 0.0,
            "vy_cmd": vy_cmd,
            "omega_cmd": 0.0,
            "duration": duration_cmd,
            "invert_vy": args.invert_vy,
            "detections_used": chosen,
        }
        records.append(rec)
        print(json.dumps(rec, ensure_ascii=False), flush=True)

        if command == "aligned":
            if client is not None:
                client.StopMove()
            break
        if args.dry_run:
            break

        motion_trace = send_velocity(client, 0.0, vy_cmd, 0.0, duration_cmd, args.motion_odom_topic)
        rec["motion_trace"] = motion_trace
        print(json.dumps({"step": step, "motion_trace": motion_trace}, ensure_ascii=False), flush=True)
        time.sleep(0.8)

    with open(out_dir / "records.json", "w") as f:
        json.dump(records, f, indent=2)
    print("records:", out_dir / "records.json", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
