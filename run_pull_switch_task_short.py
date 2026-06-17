#!/usr/bin/env python3
"""Short one-click pull-switch flow.

Expected operator flow:
1. Start the PC GUI manually.
2. Start navigation in the GUI and finish relocalization.
3. Run this script on the PC.
"""

import argparse
import json
import math
import os
import shlex
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

from g1_remote_client import G1RemoteClient


DEFAULT_ALIGN_COMMAND = """cd /home/unitree/g1_dev/yolo11 && ./start_dianzha_full_pose_align.sh \
  --max-steps 80 \
  --target-min-distance-m 1.20 \
  --target-max-distance-m 1.25 \
  --approach-deadband-deg 2.0 \
  --deadband-deg 1.5 \
  --forward-velocity 0.40 \
  --forward-duration 0.50 \
  --mid-forward-velocity 0.45 \
  --mid-forward-duration 0.50 \
  --near-forward-velocity 0.50 \
  --near-forward-duration 0.50 \
  --max-forward-duration 0.35 \
  --then-lateral \
  --max-steps 16 \
  --deadband-px 30 \
  --target-offset-px -30 \
  --min-vy 0.28 \
  --max-vy 0.32 \
  --min-duration 0.35 \
  --max-duration 0.40 \
  --then-final-yaw \
  --max-steps 12 \
  --deadband-deg 1.5 \
  --pid-step \
  --step-profile linear \
  --min-omega 0.45 \
  --max-omega 0.56 \
  --min-duration 0.50 \
  --max-duration 0.60"""

DEFAULT_PULL_COMMAND = "cd /home/unitree/zgx_g1 && ./demo_pull_switch.py --poses .runtime/pull_switch_poses.json --speak 1"
DEFAULT_LOCAL_POSES = os.path.expanduser("~/Desktop/g1_poses2.json")
REMOTE_POSES = "/home/unitree/zgx_g1/.runtime/pull_switch_poses.json"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
COLOR_TOPIC = "/camera/color/image_raw"
YOLO_TOPIC = "/yolo11/detections"


def log(msg):
    print(f"[pull-flow] {msg}", flush=True)


def default_host():
    return os.environ.get("G1_BACKEND_HOST") or os.environ.get("G1_HOST") or "10.231.138.24"


def default_waypoints():
    for path in (
        "/home/zgx/Desktop/test2.json",
        os.path.expanduser("~/Desktop/test2.json"),
        os.path.join(os.getcwd(), "test2.json"),
    ):
        if os.path.exists(path):
            return path
    return "/home/zgx/Desktop/test2.json"


def local_ip_for(host):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((host, 11311))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except Exception:
        return os.environ.get("ROS_IP", "")


def run(cmd, timeout=None, dry_run=False):
    log(f"执行: {cmd}")
    if dry_run:
        return
    subprocess.run(cmd, shell=True, check=True, timeout=timeout)


def ssh_command(host, user, command, timeout=None, dry_run=False):
    run(f"ssh {shlex.quote(user)}@{shlex.quote(host)} {shlex.quote(command)}", timeout=timeout, dry_run=dry_run)


def sync_pull_poses(host, user, local_path, dry_run=False):
    local_path = os.path.expanduser(local_path)
    if not os.path.exists(local_path):
        log(f"未找到本地拉闸 poses，跳过同步: {local_path}")
        return
    log(f"同步拉闸 poses: {local_path} -> {host}:{REMOTE_POSES}")
    ssh_command(host, user, "mkdir -p /home/unitree/zgx_g1/.runtime", timeout=5, dry_run=dry_run)
    run(
        f"rsync -az {shlex.quote(local_path)} {shlex.quote(user)}@{shlex.quote(host)}:{shlex.quote(REMOTE_POSES)}",
        timeout=10,
        dry_run=dry_run,
    )


def load_waypoints(path):
    with open(os.path.expanduser(path), encoding="utf-8") as f:
        data = json.load(f)
    items = data if isinstance(data, list) else data.get("waypoints") or data.get("points") or []
    waypoints = []
    for idx, item in enumerate(items):
        if isinstance(item, dict):
            name = item.get("name") or item.get("label") or f"航点{idx + 1}"
            x = item.get("x", 0.0)
            y = item.get("y", 0.0)
            yaw = item.get("yaw", item.get("theta", 0.0))
        else:
            name = item[0] if len(item) > 0 else f"航点{idx + 1}"
            x = item[1] if len(item) > 1 else 0.0
            y = item[2] if len(item) > 2 else 0.0
            yaw = item[3] if len(item) > 3 else 0.0
        waypoints.append({"name": str(name), "x": float(x), "y": float(y), "yaw": float(yaw)})
    if not waypoints:
        raise RuntimeError(f"航点文件为空: {path}")
    return waypoints


def nav_distance(status, waypoint):
    pose = status.get("nav_pose") if isinstance(status, dict) else None
    if not isinstance(pose, dict):
        return None
    try:
        return math.hypot(float(pose["x"]) - waypoint["x"], float(pose["y"]) - waypoint["y"])
    except Exception:
        return None


def wait_move_base_ready(host, user, timeout=25.0, dry_run=False):
    command = f"""
set -e
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://{host}:11311
export ROS_IP={host}
deadline=$((SECONDS + {int(max(1, timeout))}))
while [ "$SECONDS" -lt "$deadline" ]; do
    if rostopic info /move_base_simple/goal 2>/dev/null | awk '
        /^Subscribers:/ {{ in_sub=1; next }}
        /^Publishers:/ {{ in_sub=0 }}
        in_sub && /move_base/ {{ found=1 }}
        END {{ exit found ? 0 : 1 }}
    '; then
        exit 0
    fi
    sleep 0.5
done
echo "move_base_ready_timeout"
rostopic info /move_base_simple/goal 2>/dev/null || true
ps -ef | grep -E 'roslaunch|move_base|nav_start' | grep -v grep || true
exit 1
"""
    if dry_run:
        log("等待 move_base 就绪(dry-run)")
        return
    ssh_command(host, user, command, timeout=timeout + 8)
    log("move_base 已就绪")


def wait_navigation(client, waypoint, timeout, accept_radius, dry_run=False):
    if dry_run:
        log(f"等待导航到达(dry-run): {waypoint['name']}")
        return
    deadline = time.time() + timeout
    last_nav = None
    while time.time() < deadline:
        data = client.status().get("data", {})
        nav = data.get("nav")
        if nav != last_nav:
            log(f"导航状态: {nav}")
            last_nav = nav
        dist = nav_distance(data, waypoint)
        if nav == "succeeded":
            return
        if nav in ("aborted", "rejected", "preempted", "recalled", "stopped"):
            if dist is not None and dist <= accept_radius:
                log(f"导航 {nav}，但距离 {dist:.2f}m <= {accept_radius:.2f}m，按到达处理")
                return
            raise RuntimeError(f"{waypoint['name']} 导航失败: {nav}, dist={dist}")
        time.sleep(0.4)
    raise TimeoutError(f"{waypoint['name']} 导航超时")


def send_goal_and_wait(client, waypoint, timeout, accept_radius, dry_run=False):
    log(f"导航到 {waypoint['name']}: ({waypoint['x']:.2f}, {waypoint['y']:.2f}, {math.degrees(waypoint['yaw']):.0f}deg)")
    if not dry_run:
        client.nav_goal(waypoint["x"], waypoint["y"], waypoint["yaw"])
    wait_navigation(client, waypoint, timeout, accept_radius, dry_run=dry_run)


def stop_perception(host, user, dry_run=False):
    command = (
        "for pat in "
        "realsense2_camera rs_camera.launch start_realsense_depth_color_6fps.sh start_camera_6fps_wireless.sh "
        "yolo11_rgb_node.py start_yolo11_dianzha1_rgb.sh; do "
        "for pid in $(pgrep -f \"$pat\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; "
        "[ \"$pid\" = \"$PPID\" ] && continue; "
        "kill \"$pid\" >/dev/null 2>&1 || true; "
        "done; "
        "done; "
        "sleep 0.8"
    )
    log("清理旧相机/YOLO 进程")
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)


def kill_navigation_ros(host, user, dry_run=False):
    command = f"""
set +e
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://{host}:11311
export ROS_IP={host}
""" + r"""
timeout 2 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
timeout 2 rostopic pub -r 10 /cmd_vel_smooth geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true

for node in \
  /move_base \
  /costmap_clear \
  /velocity_smoother_ema \
  /pointcloud_to_laserscan \
  /body2any_pointcloud \
  /downsample_pointcloud \
  /map_server \
  /slam_reloc \
  /localizer_node; do
  rosnode kill "$node" >/dev/null 2>&1 || true
done

patterns=(
  "g1_nav_panel/nav_start.launch"
  "/opt/ros/noetic/lib/move_base/move_base"
  "G1Nav2D/devel/lib/fastlio/localizer_node"
  "G1Nav2D/devel/lib/fastlio/slam_reloc.py"
  "G1Nav2D/devel/lib/tool/downsample_pointcloud"
  "G1Nav2D/devel/lib/tool/body2any_pointcloud"
  "G1Nav2D/devel/lib/xju_pnc/costmap_clear"
  "G1Nav2D/devel/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node"
  "G1Nav2D/devel/lib/velocity_smoother_ema/velocity_smoother_ema_node"
  "G1Nav2D/devel/lib/livox_ros_driver2/livox_ros_driver2_node"
  "static_transform_publisher.*body base_link"
)
for pat in "${patterns[@]}"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill "$pid" >/dev/null 2>&1 || true
  done
done
sleep 1.0
for pat in "${patterns[@]}"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill -9 "$pid" >/dev/null 2>&1 || true
  done
done
"""
    log("停止并清理导航 ROS")
    ssh_command(host, user, command, timeout=20, dry_run=dry_run)


def wait_remote_topic(host, user, topic, timeout=20.0, dry_run=False):
    command = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        f"export ROS_MASTER_URI=http://{host}:11311; "
        f"export ROS_IP={host}; "
        f"timeout {int(max(1, timeout))} rostopic echo -n 1 {shlex.quote(topic)} >/dev/null"
    )
    if dry_run:
        log(f"等待 topic(dry-run): {topic}")
        return True
    try:
        ssh_command(host, user, command, timeout=timeout + 5)
        return True
    except Exception as exc:
        log(f"等待 topic 超时: {topic}: {exc}")
        return False


def start_camera(host, user, dry_run=False):
    camera_script = f"""#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://{host}:11311
export ROS_IP={host}
exec roslaunch realsense2_camera rs_camera.launch \\
  initial_reset:=true \\
  enable_depth:=true \\
  enable_color:=true \\
  align_depth:=true \\
  enable_infra:=false \\
  enable_infra1:=false \\
  enable_infra2:=false \\
  enable_gyro:=false \\
  enable_accel:=false \\
  color_width:=640 \\
  color_height:=480 \\
  color_fps:=6 \\
  depth_width:=424 \\
  depth_height:=240 \\
  depth_fps:=6
"""
    command = (
        "mkdir -p /home/unitree/zgx_g1/.runtime && "
        "for pat in realsense2_camera rs_camera.launch start_realsense_depth_color_6fps.sh start_camera_6fps_wireless.sh; do "
        "for pid in $(pgrep -f \"$pat\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; [ \"$pid\" = \"$PPID\" ] && continue; "
        "kill \"$pid\" >/dev/null 2>&1 || true; done; done; sleep 0.5; "
        f"cat > /home/unitree/zgx_g1/.runtime/start_camera_6fps_wireless.sh <<'EOF'\n{camera_script}EOF\n"
        "chmod +x /home/unitree/zgx_g1/.runtime/start_camera_6fps_wireless.sh && "
        "(setsid bash /home/unitree/zgx_g1/.runtime/start_camera_6fps_wireless.sh "
        "> /home/unitree/zgx_g1/.runtime/camera_depth_color_6fps.log 2>&1 < /dev/null &) && sleep 0.2"
    )
    log("启动相机")
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)
    if not wait_remote_topic(host, user, DEPTH_TOPIC, timeout=25.0, dry_run=dry_run):
        raise RuntimeError(f"相机未就绪: {DEPTH_TOPIC}")


def start_yolo(host, user, dry_run=False):
    command = (
        "mkdir -p /home/unitree/zgx_g1/.runtime && "
        "for pat in yolo11_rgb_node.py start_yolo11_dianzha1_rgb.sh; do "
        "for pid in $(pgrep -f \"$pat\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; [ \"$pid\" = \"$PPID\" ] && continue; "
        "kill \"$pid\" >/dev/null 2>&1 || true; done; done; sleep 0.5; "
        "cd /home/unitree/g1_dev/yolo11 && "
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 && "
        f"export ROS_MASTER_URI=http://{host}:11311 && "
        f"export ROS_IP={host} && "
        "(setsid ./start_yolo11_dianzha1_rgb.sh "
        "> /home/unitree/zgx_g1/.runtime/dianzha_yolo.log 2>&1 < /dev/null &) && sleep 0.2"
    )
    log("启动 YOLO")
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)
    if not wait_remote_topic(host, user, YOLO_TOPIC, timeout=25.0, dry_run=dry_run):
        raise RuntimeError(f"YOLO 未就绪: {YOLO_TOPIC}")


def capture_photo(topic, output_path, ros_master_uri, ros_ip, timeout, dry_run=False):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    log(f"拍照: {topic} -> {output_path}")
    if dry_run:
        return str(output_path)

    env = os.environ.copy()
    env["ROS_MASTER_URI"] = ros_master_uri
    if ros_ip:
        env["ROS_IP"] = ros_ip

    code = r"""
import os
import sys
import rospy
from sensor_msgs.msg import Image

out, topic, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3])
rospy.init_node("hongtu_capture_once_short", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message(topic, Image, timeout=timeout)

import cv2
import numpy as np

enc = msg.encoding.lower()
dtype = np.uint16 if enc in ("16uc1", "mono16") else np.uint8
channels = 3 if enc in ("rgb8", "bgr8") else 1
arr = np.frombuffer(msg.data, dtype=dtype)
if channels == 1:
    arr = arr.reshape(msg.height, msg.width)
else:
    arr = arr.reshape(msg.height, msg.width, channels)
    if enc == "rgb8":
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
if not cv2.imwrite(out, arr):
    raise RuntimeError("cv2.imwrite returned false")
print(out)
"""
    subprocess.run(
        [sys.executable, "-c", code, str(output_path), topic, str(timeout)],
        check=True,
        env=env,
        timeout=timeout + 5,
    )
    return str(output_path)


def copy_photo_dir(src, desktop_root, dry_run=False):
    src = Path(src).expanduser()
    dst = Path(desktop_root).expanduser() / src.name
    log(f"备份照片到桌面: {src} -> {dst}")
    if dry_run:
        return str(dst)
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    return str(dst)


def main():
    ap = argparse.ArgumentParser(description="Short pull-switch task flow")
    ap.add_argument("--host", default=default_host())
    ap.add_argument("--user", default="unitree")
    ap.add_argument("--waypoints", default=default_waypoints())
    ap.add_argument("--only-last-waypoint", action="store_true", help="测试：只导航到第 4 点/最后一个点")
    ap.add_argument("--nav-timeout", type=float, default=180.0)
    ap.add_argument("--accept-radius", type=float, default=0.35)
    ap.add_argument("--start-nav", action="store_true", default=True)
    ap.add_argument("--no-start-nav", dest="start_nav", action="store_false", help="GUI 已启动导航时可跳过 nav_start")
    ap.add_argument("--photo-topic", default=COLOR_TOPIC)
    ap.add_argument("--photo-timeout", type=float, default=12.0)
    ap.add_argument("--photo-dir", default=".runtime/pull_switch_task_photos")
    ap.add_argument("--desktop-photo-dir", default=os.path.expanduser("~/Desktop/pull_switch_task_photos"))
    ap.add_argument("--skip-photos", action="store_true", help="不拍照、不复制照片")
    ap.add_argument("--poses", default=DEFAULT_LOCAL_POSES, help="PC 本地拉闸动作 poses，会同步到 G1 runtime")
    ap.add_argument("--align-command", default=DEFAULT_ALIGN_COMMAND)
    ap.add_argument("--align-timeout", type=float, default=180.0)
    ap.add_argument("--pull-command", default=DEFAULT_PULL_COMMAND)
    ap.add_argument("--pull-timeout", type=float, default=90.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    waypoints = load_waypoints(args.waypoints)
    run_waypoints = [waypoints[-1]] if args.only_last_waypoint else waypoints
    if args.only_last_waypoint:
        log(f"测试模式：只导航到最后航点 {run_waypoints[-1]['name']}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    photo_dir = Path(args.photo_dir).expanduser() / f"pull_switch_{stamp}"
    ros_master_uri = f"http://{args.host}:11311"
    ros_ip = local_ip_for(args.host)
    client = G1RemoteClient(f"http://{args.host}:5055", timeout=4.0)

    log("开始短流程：导航 -> 清理导航 -> 相机/YOLO -> 微调 -> 拉闸")
    stop_perception(args.host, args.user, dry_run=args.dry_run)
    sync_pull_poses(args.host, args.user, args.poses, dry_run=args.dry_run)

    if args.start_nav:
        log("启动/确认导航")
        if not args.dry_run:
            client.nav_start()
        time.sleep(2.0)
    wait_move_base_ready(args.host, args.user, timeout=25.0, dry_run=args.dry_run)

    for waypoint in run_waypoints:
        send_goal_and_wait(client, waypoint, args.nav_timeout, args.accept_radius, dry_run=args.dry_run)

    log("已到任务点，停止导航并清理导航 ROS")
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

    stop_perception(args.host, args.user, dry_run=args.dry_run)
    start_camera(args.host, args.user, dry_run=args.dry_run)
    arrival_photo = ""
    if not args.skip_photos:
        arrival_photo = capture_photo(
            args.photo_topic,
            photo_dir / "arrival_before_align_or_pull.jpg",
            ros_master_uri,
            ros_ip,
            args.photo_timeout,
            dry_run=args.dry_run,
        )

    start_yolo(args.host, args.user, dry_run=args.dry_run)
    align_command = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        f"export ROS_MASTER_URI={shlex.quote(ros_master_uri)}; "
        f"export ROS_IP={shlex.quote(args.host)}; "
        f"{args.align_command}"
    )
    ssh_command(args.host, args.user, align_command, timeout=args.align_timeout, dry_run=args.dry_run)

    ssh_command(args.host, args.user, args.pull_command, timeout=args.pull_timeout, dry_run=args.dry_run)
    done_photo = ""
    desktop_dir = ""
    if not args.skip_photos:
        done_photo = capture_photo(
            args.photo_topic,
            photo_dir / "after_pull_done.jpg",
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
