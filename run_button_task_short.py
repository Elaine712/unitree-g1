#!/usr/bin/env python3
"""Short one-click button task flow.

Expected operator flow:
1. Start the PC GUI manually.
2. Start navigation in the GUI and finish relocalization.
3. Run this script on the PC.
"""

import argparse
import os
import shlex
import time
from pathlib import Path

from g1_remote_client import G1RemoteClient
from run_pull_switch_task_short import (
    COLOR_TOPIC,
    capture_photo,
    copy_photo_dir,
    kill_navigation_ros,
    load_waypoints,
    local_ip_for,
    log,
    send_goal_and_wait,
    ssh_command,
    start_camera,
    stop_perception,
    wait_move_base_ready,
)


DEFAULT_BUTTON_COMMAND = "cd /home/unitree && ./start_task1_full_serial.sh"


def default_host():
    return os.environ.get("G1_BACKEND_HOST") or os.environ.get("G1_HOST") or "10.231.138.24"


def default_waypoints():
    for path in (
        "/home/zgx/Desktop/test3.json",
        os.path.expanduser("~/Desktop/test3.json"),
        os.path.join(os.getcwd(), "test3.json"),
        "/home/zgx/Desktop/test3.josn",
        os.path.expanduser("~/Desktop/test3.josn"),
    ):
        if os.path.exists(path):
            return path
    return "/home/zgx/Desktop/test3.json"


def stop_button_processes(host, user, dry_run=False):
    command = (
        "for pat in "
        "start_task1_full_serial.sh start_all_switch_align.sh run_full_flow.sh "
        "switch_align button_press HJQ_1 yolo11_rgb_node.py realsense2_camera rs_camera.launch; do "
        "for pid in $(pgrep -f \"$pat\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; "
        "[ \"$pid\" = \"$PPID\" ] && continue; "
        "kill \"$pid\" >/dev/null 2>&1 || true; "
        "done; "
        "done; "
        "sleep 0.8"
    )
    log("清理旧相机/视觉/按钮流程进程")
    ssh_command(host, user, command, timeout=10, dry_run=dry_run)


def main():
    ap = argparse.ArgumentParser(description="Short button task flow")
    ap.add_argument("--host", default=default_host())
    ap.add_argument("--user", default="unitree")
    ap.add_argument("--waypoints", default=default_waypoints(), help="按钮任务点位文件，默认桌面 test3.json")
    ap.add_argument("--nav-timeout", type=float, default=180.0)
    ap.add_argument("--accept-radius", type=float, default=0.35)
    ap.add_argument("--start-nav", action="store_true", default=True)
    ap.add_argument("--no-start-nav", dest="start_nav", action="store_false", help="GUI 已启动导航时可跳过 nav_start")
    ap.add_argument("--photo-topic", default=COLOR_TOPIC)
    ap.add_argument("--photo-timeout", type=float, default=12.0)
    ap.add_argument("--photo-dir", default=".runtime/button_task_photos")
    ap.add_argument("--desktop-photo-dir", default=os.path.expanduser("~/Desktop/button_task_photos"))
    ap.add_argument("--skip-photos", action="store_true", help="不拍照、不复制照片")
    ap.add_argument("--button-command", default=DEFAULT_BUTTON_COMMAND)
    ap.add_argument("--button-timeout", type=float, default=300.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    waypoints = load_waypoints(args.waypoints)
    if len(waypoints) != 2:
        log(f"点位文件包含 {len(waypoints)} 个点；按钮任务预期 2 个点，将按文件顺序全部执行")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    photo_dir = Path(args.photo_dir).expanduser() / f"button_task_{stamp}"
    ros_master_uri = f"http://{args.host}:11311"
    ros_ip = local_ip_for(args.host)
    client = G1RemoteClient(f"http://{args.host}:5055", timeout=4.0)

    log("开始按钮短流程：两点导航 -> 清理导航 -> 本体视觉微调+按按钮")
    stop_button_processes(args.host, args.user, dry_run=args.dry_run)
    stop_perception(args.host, args.user, dry_run=args.dry_run)

    if args.start_nav:
        log("启动/确认导航")
        if not args.dry_run:
            client.nav_start()
        time.sleep(2.0)
    wait_move_base_ready(args.host, args.user, timeout=25.0, dry_run=args.dry_run)

    for waypoint in waypoints:
        send_goal_and_wait(client, waypoint, args.nav_timeout, args.accept_radius, dry_run=args.dry_run)

    log("已到按钮任务点，停止导航并清理导航 ROS")
    if not args.dry_run:
        try:
            client.nav_stop()
        except Exception as exc:
            log(f"nav_stop 失败，继续强制清理: {exc}")
        try:
            client.stop()
        except Exception:
            pass
    kill_navigation_ros(args.host, args.user, dry_run=args.dry_run)

    arrival_photo = ""
    done_photo = ""
    desktop_dir = ""
    if not args.skip_photos:
        start_camera(args.host, args.user, dry_run=args.dry_run)
        arrival_photo = capture_photo(
            args.photo_topic,
            photo_dir / "arrival_before_button.jpg",
            ros_master_uri,
            ros_ip,
            args.photo_timeout,
            dry_run=args.dry_run,
        )

    command = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        f"export ROS_MASTER_URI={shlex.quote(ros_master_uri)}; "
        f"export ROS_IP={shlex.quote(args.host)}; "
        f"{args.button_command}"
    )
    ssh_command(args.host, args.user, command, timeout=args.button_timeout, dry_run=args.dry_run)

    if not args.skip_photos:
        done_photo = capture_photo(
            args.photo_topic,
            photo_dir / "after_button_done.jpg",
            ros_master_uri,
            ros_ip,
            args.photo_timeout,
            dry_run=args.dry_run,
        )
        desktop_dir = copy_photo_dir(photo_dir, args.desktop_photo_dir, dry_run=args.dry_run)

    if args.skip_photos:
        log("完成。已跳过拍照和桌面备份")
    else:
        log(f"完成。到点照片: {arrival_photo}")
        log(f"完成后照片: {done_photo}")
        log(f"桌面备份: {desktop_dir}")


if __name__ == "__main__":
    main()
