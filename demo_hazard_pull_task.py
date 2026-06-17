#!/usr/bin/env python3
"""One-click hazard handling demo: alarm -> patrol -> align -> photos -> pull switch."""

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

from g1_remote_client import G1RemoteClient, G1RemoteError


class NavigationFailed(RuntimeError):
    def __init__(self, status, distance, target):
        super().__init__(f"导航失败: {status}, dist={distance}")
        self.status = status
        self.distance = distance
        self.target = target


DEFAULT_ALIGN_COMMAND = """cd /home/unitree/g1_dev/yolo11 && ./start_dianzha_full_pose_align.sh \
  --max-steps 80 \
  --target-min-distance-m 1.25 \
  --target-max-distance-m 1.30 \
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
  --min-omega 0.45 \
  --max-omega 0.56 \
  --min-duration 0.50 \
  --max-duration 0.60"""


DEFAULT_PULL_COMMAND = (
    "cd /home/unitree/zgx_g1 && "
    "./demo_pull_switch.py --speak 1"
)


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


def now_stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def log(msg):
    print(f"[hazard] {msg}", flush=True)


def local_ip_for(host):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((host, 11311))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return os.environ.get("ROS_IP", "")


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


def run(cmd, timeout=None, dry_run=False):
    log(f"执行: {cmd}")
    if dry_run:
        return
    subprocess.run(cmd, shell=True, check=True, timeout=timeout)


def ssh_command(host, user, command, timeout=None, dry_run=False):
    quoted = shlex.quote(command)
    run(f"ssh {shlex.quote(user)}@{shlex.quote(host)} {quoted}", timeout=timeout, dry_run=dry_run)


def ensure_backend(host, user, dry_run=False):
    command = (
        "cd /home/unitree/zgx_g1 && mkdir -p .runtime && "
        "if ! ss -ltn | grep -q ':5055 '; then "
        "nohup ./start_g1_backend.sh > .runtime/g1_backend.log 2>&1 < /dev/null & "
        "fi"
    )
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)
    if dry_run:
        return
    deadline = time.time() + 25
    client = G1RemoteClient(f"http://{host}:5055", timeout=2.0)
    last = None
    while time.time() < deadline:
        try:
            client.status()
            log("G1 后台服务已就绪")
            return
        except Exception as e:
            last = e
            time.sleep(0.8)
    raise RuntimeError(f"G1 后台服务未就绪: {last}")


def start_realsense(
    host,
    user,
    depth_width=424,
    depth_height=240,
    depth_fps=6,
    color_width=640,
    color_height=480,
    color_fps=6,
    initial_reset=True,
    dry_run=False,
):
    reset_arg = "initial_reset:=true \\\n  " if initial_reset else ""
    camera_script = f"""#!/usr/bin/env bash
set -e

export ROS_IP={host}
export ROS_MASTER_URI=http://{host}:11311
source /opt/ros/noetic/setup.bash

exec roslaunch realsense2_camera rs_camera.launch \\
  {reset_arg}enable_depth:=true \\
  enable_color:=true \\
  align_depth:=true \\
  enable_infra:=false \\
  enable_infra1:=false \\
  enable_infra2:=false \\
  enable_gyro:=false \\
  enable_accel:=false \\
  color_width:={int(color_width)} \\
  color_height:={int(color_height)} \\
  color_fps:={int(color_fps)} \\
  depth_width:={int(depth_width)} \\
  depth_height:={int(depth_height)} \\
  depth_fps:={int(depth_fps)}
"""
    command = (
        "mkdir -p /home/unitree/zgx_g1/.runtime && "
        "for pat in realsense2_camera rs_camera.launch start_realsense_depth_color_6fps.sh start_camera_6fps_wireless.sh; do "
        "for pid in $(pgrep -f \"$pat\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; "
        "[ \"$pid\" = \"$PPID\" ] && continue; "
        "kill \"$pid\" >/dev/null 2>&1 || true; "
        "done; "
        "done; "
        "sleep 0.5; "
        f"cat > /home/unitree/zgx_g1/.runtime/start_camera_6fps_wireless.sh <<'EOF'\n{camera_script}EOF\n"
        "chmod +x /home/unitree/zgx_g1/.runtime/start_camera_6fps_wireless.sh && "
        "(setsid bash /home/unitree/zgx_g1/.runtime/start_camera_6fps_wireless.sh "
        "> /home/unitree/zgx_g1/.runtime/camera_depth_color_6fps.log 2>&1 < /dev/null &) && "
        "sleep 0.2; exit 0"
    )
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)


def wait_remote_topic(host, user, topic, timeout=20.0, dry_run=False):
    command = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        f"timeout {int(max(1, timeout))} rostopic echo -n 1 {shlex.quote(topic)} >/dev/null"
    )
    if dry_run:
        log(f"等待 G1 topic(dry-run): {topic}")
        return True
    try:
        ssh_command(host, user, command, timeout=timeout + 5, dry_run=False)
        return True
    except Exception as e:
        log(f"等待 topic 超时: {topic}: {e}")
        return False


def wait_move_base_ready(host, user, timeout=25.0, dry_run=False):
    command = f"""
set -e
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
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
echo "--- package check ---"
rospack find move_base || true
ls -l /opt/ros/noetic/lib/move_base/move_base 2>/dev/null || true
echo "--- goal topic ---"
rostopic info /move_base_simple/goal 2>/dev/null || true
echo "--- processes ---"
ps -ef | grep -E 'roslaunch|move_base|nav_start' | grep -v grep || true
exit 1
"""
    if dry_run:
        log("等待 move_base goal 订阅者(dry-run)")
        return
    try:
        ssh_command(host, user, command, timeout=timeout + 8, dry_run=False)
        log("move_base 已就绪：/move_base_simple/goal 有订阅者")
    except Exception as e:
        raise RuntimeError(
            "move_base 未就绪：/move_base_simple/goal 没有 move_base 订阅者。"
            "请先在 G1 上安装 ros-noetic-move-base，或检查导航 launch 日志。"
        ) from e


def start_realsense_until_ready(
    host,
    user,
    depth_topic,
    attempts=3,
    wait_timeout=20.0,
    depth_width=424,
    depth_height=240,
    depth_fps=6,
    initial_reset=True,
    dry_run=False,
):
    for attempt in range(1, max(1, int(attempts)) + 1):
        log(f"启动深度相机({attempt}/{attempts})")
        start_realsense(
            host,
            user,
            depth_width=depth_width,
            depth_height=depth_height,
            depth_fps=depth_fps,
            initial_reset=initial_reset,
            dry_run=dry_run,
        )
        if wait_remote_topic(host, user, depth_topic, timeout=wait_timeout, dry_run=dry_run):
            log(f"深度相机已就绪: {depth_topic}")
            return
        log("深度图未就绪，准备重启相机")
    raise RuntimeError(f"深度相机未发布 topic: {depth_topic}")


def start_dianzha_yolo(host, user, dry_run=False):
    command = (
        "mkdir -p /home/unitree/zgx_g1/.runtime && "
        "for pat in yolo11_rgb_node.py start_yolo11_dianzha1_rgb.sh; do "
        "for pid in $(pgrep -f \"$pat\" 2>/dev/null || true); do "
        "[ \"$pid\" = \"$$\" ] && continue; "
        "[ \"$pid\" = \"$PPID\" ] && continue; "
        "kill \"$pid\" >/dev/null 2>&1 || true; "
        "done; "
        "done; "
        "sleep 0.5; "
        "cd /home/unitree/g1_dev/yolo11 && "
        "(setsid ./start_yolo11_dianzha1_rgb.sh "
        "> /home/unitree/zgx_g1/.runtime/dianzha_yolo.log 2>&1 < /dev/null &) && "
        "sleep 0.2; exit 0"
    )
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)


def start_dianzha_yolo_until_ready(host, user, attempts=2, wait_timeout=20.0, dry_run=False):
    topic = "/yolo11/detections"
    for attempt in range(1, max(1, int(attempts)) + 1):
        log(f"启动电闸 YOLO({attempt}/{attempts})")
        start_dianzha_yolo(host, user, dry_run=dry_run)
        if wait_remote_topic(host, user, topic, timeout=wait_timeout, dry_run=dry_run):
            log(f"电闸 YOLO 已就绪: {topic}")
            return
        log("电闸 YOLO 检测消息未就绪，准备重启 YOLO")
    raise RuntimeError(f"电闸 YOLO 未发布检测消息: {topic}")


def stop_perception_ros(host, user, dry_run=False):
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
    log("导航前清理旧相机/YOLO ROS，避免旧 ROS master 干扰导航")
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)


def dump_align_debug_snapshot(host, user, label, dry_run=False):
    command = r"""
set +e
echo "=== align_debug %s ==="
date '+time=%F %T'
echo "--- backend status ---"
python3 - <<'PY'
import json, urllib.request
try:
    raw = urllib.request.urlopen("http://127.0.0.1:5055/api/status", timeout=2).read().decode()
    data = json.loads(raw).get("data", {})
    keep = {k: data.get(k) for k in ["ready", "moving", "nav", "nav_running", "nav_pose", "nav_last_goal"]}
    print(json.dumps(keep, ensure_ascii=False))
except Exception as e:
    print("backend_status_error:", e)
PY
echo "--- nav/camera/yolo processes ---"
ps -ef | grep -E 'move_base|velocity_smoother|g1_robot_service|g1_wall_black|g1_dianzha|yolo11_rgb_node|rs_camera|realsense2_camera' | grep -v grep || true
echo "--- ros topics one-shot ---"
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
timeout 2 rostopic echo -n 1 /cmd_vel 2>/dev/null | sed -n '1,20p' || echo 'cmd_vel: no message'
timeout 2 rostopic echo -n 1 /cmd_vel_smooth 2>/dev/null | sed -n '1,20p' || echo 'cmd_vel_smooth: no message'
timeout 2 rostopic echo -n 1 /slam_odom 2>/dev/null | sed -n '1,38p' || echo 'slam_odom: no message'
""".replace("%s", label)
    log(f"采集微调诊断快照: {label}")
    ssh_command(host, user, command, timeout=15, dry_run=dry_run)


def cancel_move_base_goal(host, user, dry_run=False):
    command = r"""
set -e
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{stamp: {secs: 0, nsecs: 0}, id: ''}" >/dev/null
"""
    log("取消当前 move_base goal，保留导航栈在线")
    ssh_command(host, user, command, timeout=8, dry_run=dry_run)


def wait_cmd_vel_zero(host, user, quiet_window=0.8, timeout=8.0, dry_run=False):
    command = f"""
set -e
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
python3 - <<'PY'
import time
import rospy
from geometry_msgs.msg import Twist

quiet_window = {float(quiet_window)!r}
timeout = {float(timeout)!r}
last_nonzero = time.time()
last_msg = None
samples = 0

def cb(msg):
    global last_nonzero, last_msg, samples
    samples += 1
    last_msg = (msg.linear.x, msg.linear.y, msg.angular.z)
    if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3:
        last_nonzero = time.time()

rospy.init_node("hongtu_wait_cmd_vel_zero", anonymous=True, disable_signals=True)
sub = rospy.Subscriber("/cmd_vel", Twist, cb, queue_size=10)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
deadline = time.time() + timeout
zero = Twist()
rate = rospy.Rate(10)
while time.time() < deadline and not rospy.is_shutdown():
    pub.publish(zero)
    if time.time() - last_nonzero >= quiet_window:
        print("cmd_vel_zero_ok samples=%d last=%s" % (samples, last_msg))
        break
    rate.sleep()
else:
    raise SystemExit("cmd_vel did not stay zero within %.1fs; samples=%d last=%s" % (timeout, samples, last_msg))
PY
"""
    log(f"等待 /cmd_vel 连续归零 {quiet_window:.1f}s")
    ssh_command(host, user, command, timeout=timeout + 5, dry_run=dry_run)


def kill_navigation_ros(host, user, dry_run=False):
    command = r"""
set +e
source /opt/ros/noetic/setup.bash >/dev/null 2>&1

echo "--- publish zero cmd_vel before nav cleanup ---"
timeout 2 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
timeout 2 rostopic pub -r 10 /cmd_vel_smooth geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true

echo "--- kill navigation ros nodes ---"
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

echo "--- kill navigation processes ---"
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

echo "--- remaining nav processes ---"
ps -ef | grep -E 'move_base|nav_start|localizer_node|slam_reloc|velocity_smoother|pointcloud_to_laserscan|body2any_pointcloud|downsample_pointcloud|livox_ros_driver2' | grep -v grep || true
echo "--- remaining nav topics ---"
rostopic list 2>/dev/null | grep -E '^/(move_base|cmd_vel_smooth|slam_odom|scan|body_cloud|base_link_cloud|livox)' || true
"""
    log("强制清理导航 ROS 节点和进程，进入微调前干净状态")
    ssh_command(host, user, command, timeout=20, dry_run=dry_run)


def update_task(client, stage, note="", dry_run=False, **data):
    payload = {"stage": stage}
    if note:
        payload["note"] = note
    payload.update(data)
    if dry_run:
        log(f"任务状态(dry-run): {payload}")
        return
    try:
        client.task_update(**payload)
    except Exception as e:
        log(f"任务状态更新失败（忽略）: {e}")


def nav_distance(data, target):
    pose = data.get("nav_pose") if isinstance(data, dict) else None
    if not isinstance(pose, dict):
        return None
    try:
        return math.hypot(float(pose.get("x")) - target["x"], float(pose.get("y")) - target["y"])
    except Exception:
        return None


def wait_navigation(client, target, timeout, accept_radius):
    end = time.time() + timeout
    last_nav = None
    while time.time() < end:
        data = client.status().get("data", {})
        nav = data.get("nav")
        if nav != last_nav:
            log(f"导航状态: {nav}")
            last_nav = nav
        dist = nav_distance(data, target)
        if nav == "succeeded":
            return True
        if nav in ("aborted", "rejected", "preempted", "recalled", "stopped"):
            if dist is not None and dist <= accept_radius:
                log(f"导航 {nav}，但距离 {dist:.2f}m <= {accept_radius:.2f}m，按到达处理")
                return True
            raise NavigationFailed(nav, dist, target)
        time.sleep(0.4)
    raise TimeoutError(f"导航超时: {target['name']}")


def send_goal_and_wait(
    client,
    waypoint,
    timeout,
    accept_radius,
    adjust,
    retries=3,
    retry_delay=1.2,
    continue_on_fail=False,
    dry_run=False,
):
    log(f"导航到 {waypoint['name']}: ({waypoint['x']:.2f}, {waypoint['y']:.2f}, {math.degrees(waypoint['yaw']):.0f}deg)")
    if dry_run:
        return
    attempts = max(1, int(retries) + 1)
    last_error = None
    for attempt in range(1, attempts + 1):
        if attempt > 1:
            log(f"重新下发 {waypoint['name']} ({attempt}/{attempts})")
        try:
            client.nav_goal(waypoint["x"], waypoint["y"], waypoint["yaw"], adjust=adjust)
        except TypeError as exc:
            if "adjust" not in str(exc):
                raise
            client.nav_goal(waypoint["x"], waypoint["y"], waypoint["yaw"])
        try:
            wait_navigation(client, waypoint, timeout, accept_radius)
            if attempt > 1:
                log(f"{waypoint['name']} 重试后到达")
            return
        except (NavigationFailed, TimeoutError) as e:
            last_error = e
            if attempt >= attempts:
                break
            log(f"{waypoint['name']} 本次导航失败，{retry_delay:.1f}s 后重试: {e}")
            time.sleep(max(0.0, retry_delay))
    if continue_on_fail:
        log(f"{waypoint['name']} 多次失败，按参数要求继续后续任务: {last_error}")
        return
    raise last_error


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
rospy.init_node("hongtu_capture_once", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message(topic, Image, timeout=timeout)

try:
    import cv2
    import numpy as np
    enc = msg.encoding.lower()
    dtype = np.uint16 if enc in ("16uc1", "mono16") else np.uint8
    channels = 1
    if enc in ("rgb8", "bgr8"):
        channels = 3
    arr = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        arr = arr.reshape(msg.height, msg.width)
    else:
        arr = arr.reshape(msg.height, msg.width, channels)
        if enc == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(out, arr):
        raise RuntimeError("cv2.imwrite returned false")
except Exception:
    if msg.encoding.lower() not in ("rgb8", "bgr8", "mono8"):
        raise
    data = bytes(msg.data)
    with open(out, "wb") as f:
        if msg.encoding.lower() == "mono8":
            f.write(("P5\n%d %d\n255\n" % (msg.width, msg.height)).encode())
            f.write(data)
        else:
            if msg.encoding.lower() == "bgr8":
                data = b"".join(data[i+2:i+3] + data[i+1:i+2] + data[i:i+1] for i in range(0, len(data), 3))
            f.write(("P6\n%d %d\n255\n" % (msg.width, msg.height)).encode())
            f.write(data)
print(out)
"""
    subprocess.run(
        [sys.executable, "-c", code, str(output_path), topic, str(timeout)],
        check=True,
        env=env,
        timeout=timeout + 5,
    )
    return str(output_path)


def copy_photo_bundle(photo_run_dir, desktop_dir, dry_run=False):
    if not desktop_dir:
        return ""
    src = Path(photo_run_dir).expanduser()
    dst_root = Path(desktop_dir).expanduser()
    dst = dst_root / src.name
    log(f"复制照片文件夹到桌面: {src} -> {dst}")
    if dry_run:
        return str(dst)
    if not src.exists():
        log(f"照片文件夹不存在，跳过复制: {src}")
        return ""
    if dst.exists():
        shutil.rmtree(dst)
    dst_root.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, dst)
    return str(dst)


def main():
    ap = argparse.ArgumentParser(description="HongTu/G1 industrial hazard pull-switch closed-loop demo")
    ap.add_argument("--host", default=default_host(), help="G1 Ubuntu IP, e.g. 10.231.138.24 or 192.168.123.164")
    ap.add_argument("--user", default=os.environ.get("G1_USER", "unitree"))
    ap.add_argument("--waypoints", default=default_waypoints())
    ap.add_argument("--nav-timeout", type=float, default=180.0)
    ap.add_argument("--nav-retries", type=int, default=3, help="每个航点失败后重新下发的次数")
    ap.add_argument("--nav-retry-delay", type=float, default=1.2)
    ap.add_argument("--continue-on-nav-fail", action="store_true", help="航点多次失败后继续后续流程；实控默认关闭")
    ap.add_argument("--accept-radius", type=float, default=0.18)
    ap.add_argument("--first-accept-radius", type=float, default=0.30, help="第一个航点通常用于起步校验，可放宽到达半径")
    ap.add_argument("--return-accept-radius", type=float, default=0.30, help="返航到第一个航点的到达半径")
    ap.set_defaults(nav_adjust=False)
    ap.add_argument("--nav-adjust", dest="nav_adjust", action="store_true", help="对航点启用后台 backoff/lateral 补偿；默认关闭")
    ap.add_argument("--ensure-backend", action="store_true", help="SSH start backend if 5055 is not listening")
    ap.add_argument("--start-nav", action="store_true", help="call /api/nav/start before waypoint navigation")
    ap.add_argument(
        "--align-release-mode",
        choices=("kill_nav", "cancel_goal", "nav_stop", "none"),
        default="kill_nav",
        help="视觉微调前释放导航速度的方式；默认杀掉导航 ROS，进入干净微调状态",
    )
    ap.set_defaults(stop_nav_before_align=False)
    ap.add_argument("--stop-nav-before-align", dest="stop_nav_before_align", action="store_true", help="兼容旧参数：视觉微调前停止整个导航栈")
    ap.add_argument("--no-stop-nav-before-align", dest="stop_nav_before_align", action="store_false")
    ap.add_argument("--cmd-vel-zero-window", type=float, default=0.8, help="微调前要求 /cmd_vel 连续归零的时间")
    ap.add_argument("--cmd-vel-zero-timeout", type=float, default=8.0, help="等待 /cmd_vel 归零的超时时间")
    ap.add_argument("--restart-nav-before-return", action="store_true", help="返航前强制重启导航；默认在停止过导航时自动重启")
    ap.add_argument("--start-camera", action="store_true", help="SSH start realsense roslaunch on G1")
    ap.add_argument("--camera-start-attempts", type=int, default=3)
    ap.add_argument("--camera-ready-timeout", type=float, default=20.0)
    ap.add_argument("--camera-depth-width", type=int, default=424, help="RealSense depth width; 424 avoids 640x480 depth startup failures on the current D435I")
    ap.add_argument("--camera-depth-height", type=int, default=240)
    ap.add_argument("--camera-depth-fps", type=int, default=6)
    ap.set_defaults(camera_initial_reset=True)
    ap.add_argument("--camera-initial-reset", dest="camera_initial_reset", action="store_true")
    ap.add_argument("--no-camera-initial-reset", dest="camera_initial_reset", action="store_false")
    ap.add_argument("--skip-align", action="store_true")
    ap.add_argument("--align-command", default=DEFAULT_ALIGN_COMMAND)
    ap.add_argument("--align-timeout", type=float, default=180.0)
    ap.set_defaults(start_yolo=True)
    ap.add_argument("--start-yolo", dest="start_yolo", action="store_true", help="微调前启动电闸 YOLO；默认开启")
    ap.add_argument("--no-start-yolo", dest="start_yolo", action="store_false", help="不自动启动电闸 YOLO")
    ap.add_argument("--yolo-start-attempts", type=int, default=2)
    ap.add_argument("--yolo-ready-timeout", type=float, default=20.0)
    ap.add_argument("--skip-photos", action="store_true")
    ap.add_argument("--photo-topic", default="/camera/color/image_raw")
    ap.add_argument("--photo-dir", default=os.path.join(os.getcwd(), ".runtime", "hazard_task_photos"))
    ap.add_argument("--photo-folder-prefix", default="hazard_pull")
    ap.add_argument("--desktop-photo-dir", default=os.path.expanduser("~/Desktop"), help="任务照片完成后整文件夹复制到该目录；传空字符串可关闭")
    ap.add_argument("--photo-timeout", type=float, default=8.0)
    ap.add_argument("--ros-master-uri", default="")
    ap.add_argument("--ros-ip", default=os.environ.get("ROS_IP", ""))
    ap.add_argument("--skip-pull", action="store_true")
    ap.add_argument("--pull-command", default=DEFAULT_PULL_COMMAND)
    ap.add_argument("--pull-timeout", type=float, default=180.0)
    ap.set_defaults(return_first=False)
    ap.add_argument("--return-first", dest="return_first", action="store_true")
    ap.add_argument("--no-return-first", dest="return_first", action="store_false")
    ap.add_argument("--only-last-waypoint", action="store_true", help="测试模式：跳过前置航点，只下发航点文件中的最后一个航点")
    ap.add_argument("--last-with-previous", action="store_true", help="测试模式：只执行倒数第二个航点和最后航点，适合短流程测试最后动作点")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    waypoints = load_waypoints(args.waypoints)
    if args.last_with_previous:
        if len(waypoints) < 2:
            raise SystemExit("--last-with-previous 需要航点文件至少包含 2 个航点")
        run_waypoints = waypoints[-2:]
        log(f"测试模式：只执行最后 2 个航点 {run_waypoints[0]['name']} -> {run_waypoints[-1]['name']}")
        if args.return_first:
            args.return_first = False
            log("短流程测试默认关闭返航；如需返航请不要使用 --last-with-previous")
    elif args.only_last_waypoint:
        run_waypoints = [waypoints[-1]]
        log(f"测试模式：跳过前 {len(waypoints) - 1} 个航点，只执行最后航点 {waypoints[-1]['name']}")
        if len(waypoints) > 1:
            prev_wp = waypoints[-2]
            dist_from_prev = math.hypot(waypoints[-1]["x"] - prev_wp["x"], waypoints[-1]["y"] - prev_wp["y"])
            log(f"提示：只下发最后航点会从当前位置直接规划到目标；若距离较远或路径绕障，请改用 --last-with-previous（倒数两点距离约 {dist_from_prev:.1f}m）")
        if args.return_first:
            args.return_first = False
            log("测试模式默认关闭返航；如需返航请不要使用 --only-last-waypoint")
    else:
        run_waypoints = waypoints
    stamp = now_stamp()
    photo_run_dir = Path(args.photo_dir).expanduser() / f"{args.photo_folder_prefix}_{stamp}"
    client = G1RemoteClient(f"http://{args.host}:5055", timeout=4.0)
    ros_master_uri = args.ros_master_uri or f"http://{args.host}:11311"
    ros_ip = args.ros_ip or local_ip_for(args.host)
    waypoint_photos = []
    action_before_path = ""
    action_after_path = ""
    release_mode = "nav_stop" if args.stop_nav_before_align else args.align_release_mode
    defer_camera_until_align = args.start_camera and release_mode == "kill_nav"

    log("收到危险指令：生产厂房B疑似漏电，开始自主巡检处置闭环")
    if args.ensure_backend:
        ensure_backend(args.host, args.user, dry_run=args.dry_run)
    if defer_camera_until_align:
        stop_perception_ros(args.host, args.user, dry_run=args.dry_run)
        log("kill_nav 模式下相机延后到导航清理后、微调前启动")
    elif args.start_camera:
        start_realsense_until_ready(
            args.host,
            args.user,
            "/camera/aligned_depth_to_color/image_raw",
            attempts=args.camera_start_attempts,
            wait_timeout=args.camera_ready_timeout,
            depth_width=args.camera_depth_width,
            depth_height=args.camera_depth_height,
            depth_fps=args.camera_depth_fps,
            initial_reset=args.camera_initial_reset,
            dry_run=args.dry_run,
        )

    if not args.dry_run:
        client.task_reset()
    update_task(
        client,
        "alarm",
        "收到危险指令：生产厂房B疑似漏电，机器人开始前往处置",
        dry_run=args.dry_run,
        alarm="leakage",
        target="生产厂房B电闸",
    )
    if not args.dry_run:
        try:
            client.speak("收到漏电告警，开始前往生产厂房B进行处置")
        except Exception:
            pass

    if args.start_nav:
        log("启动本体导航")
        if not args.dry_run:
            client.nav_start()
        time.sleep(2.0)
        wait_move_base_ready(args.host, args.user, timeout=25.0, dry_run=args.dry_run)

    update_task(client, "navigating", "按航点前往目标区域", dry_run=args.dry_run, nav_goal=run_waypoints[-1])
    for idx, wp in enumerate(run_waypoints):
        radius = args.first_accept_radius if idx == 0 else args.accept_radius
        is_action_point = idx == len(run_waypoints) - 1
        send_goal_and_wait(
            client,
            wp,
            args.nav_timeout,
            radius,
            args.nav_adjust,
            retries=args.nav_retries,
            retry_delay=args.nav_retry_delay,
            continue_on_fail=args.continue_on_nav_fail,
            dry_run=args.dry_run,
        )
        if not args.skip_photos and not is_action_point:
            photo_path = capture_photo(
                args.photo_topic,
                photo_run_dir / f"waypoint_{idx + 1:02d}_arrival.jpg",
                ros_master_uri,
                ros_ip,
                args.photo_timeout,
                dry_run=args.dry_run,
            )
            waypoint_photos.append({"waypoint": wp["name"], "photo": photo_path})
            update_task(
                client,
                "checkpoint_photo",
                f"{wp['name']} 到达拍照完成",
                dry_run=args.dry_run,
                waypoint=wp,
                photos={"waypoints": waypoint_photos},
            )

    if not args.skip_align:
        update_task(client, "aligning", "到达末端航点，执行视觉位置微调", dry_run=args.dry_run)
        nav_was_stopped_for_align = release_mode == "nav_stop"
        if release_mode == "kill_nav":
            nav_was_stopped_for_align = True
            log("视觉微调前彻底停止导航并清理导航 ROS，避免导航影响微调")
            if not args.dry_run:
                try:
                    client.nav_stop()
                except Exception as e:
                    log(f"后台停止导航失败（继续强制清理导航 ROS）: {e}")
                try:
                    client.stop()
                except Exception:
                    pass
            kill_navigation_ros(args.host, args.user, dry_run=args.dry_run)
        elif release_mode == "nav_stop":
            log("视觉微调前停止导航，释放本体速度控制")
            if not args.dry_run:
                try:
                    client.nav_stop()
                except Exception as e:
                    log(f"停止导航失败（继续尝试微调）: {e}")
                try:
                    client.stop()
                except Exception:
                    pass
                time.sleep(1.0)
        elif release_mode == "cancel_goal":
            cancel_move_base_goal(args.host, args.user, dry_run=args.dry_run)
            wait_cmd_vel_zero(
                args.host,
                args.user,
                quiet_window=args.cmd_vel_zero_window,
                timeout=args.cmd_vel_zero_timeout,
                dry_run=args.dry_run,
            )
        else:
            log("视觉微调前不释放导航 goal，仅采集诊断并继续")
        dump_align_debug_snapshot(args.host, args.user, "before_align", dry_run=args.dry_run)
        if defer_camera_until_align:
            start_realsense_until_ready(
                args.host,
                args.user,
                "/camera/aligned_depth_to_color/image_raw",
                attempts=args.camera_start_attempts,
                wait_timeout=args.camera_ready_timeout,
                depth_width=args.camera_depth_width,
                depth_height=args.camera_depth_height,
                depth_fps=args.camera_depth_fps,
                initial_reset=args.camera_initial_reset,
                dry_run=args.dry_run,
            )
        elif args.start_camera:
            if wait_remote_topic(
                args.host,
                args.user,
                "/camera/aligned_depth_to_color/image_raw",
                timeout=args.camera_ready_timeout,
                dry_run=args.dry_run,
            ):
                log("微调前深度相机仍在线")
            else:
                start_realsense_until_ready(
                    args.host,
                    args.user,
                    "/camera/aligned_depth_to_color/image_raw",
                    attempts=args.camera_start_attempts,
                    wait_timeout=args.camera_ready_timeout,
                    depth_width=args.camera_depth_width,
                    depth_height=args.camera_depth_height,
                    depth_fps=args.camera_depth_fps,
                    initial_reset=args.camera_initial_reset,
                    dry_run=args.dry_run,
                )
        if args.start_yolo:
            start_dianzha_yolo_until_ready(
                args.host,
                args.user,
                attempts=args.yolo_start_attempts,
                wait_timeout=args.yolo_ready_timeout,
                dry_run=args.dry_run,
            )
        align_command = (
            "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
            f"export ROS_MASTER_URI={shlex.quote(f'http://{args.host}:11311')}; "
            f"export ROS_IP={shlex.quote(args.host)}; "
            f"{args.align_command}"
        )
        ssh_command(args.host, args.user, align_command, timeout=args.align_timeout, dry_run=args.dry_run)
        dump_align_debug_snapshot(args.host, args.user, "after_align", dry_run=args.dry_run)
    else:
        nav_was_stopped_for_align = False

    if not args.skip_photos:
        action_wp = run_waypoints[-1]
        action_before_path = capture_photo(
            args.photo_topic,
            photo_run_dir / f"waypoint_{len(waypoints):02d}_action_before.jpg",
            ros_master_uri,
            ros_ip,
            args.photo_timeout,
            dry_run=args.dry_run,
        )
        waypoint_photos.append({"waypoint": action_wp["name"], "photo": action_before_path, "role": "action_before"})
        update_task(
            client,
            "detecting",
            "动作任务点到达，操作前拍照完成",
            dry_run=args.dry_run,
            waypoint=action_wp,
            photos={"waypoints": waypoint_photos, "action_before": action_before_path},
        )

    if not args.skip_pull:
        update_task(client, "operating", "执行拉闸一键脚本", dry_run=args.dry_run)
        ssh_command(args.host, args.user, args.pull_command, timeout=args.pull_timeout, dry_run=args.dry_run)

    if not args.skip_photos and not args.skip_pull:
        action_after_path = capture_photo(
            args.photo_topic,
            photo_run_dir / f"waypoint_{len(waypoints):02d}_action_after.jpg",
            ros_master_uri,
            ros_ip,
            args.photo_timeout,
            dry_run=args.dry_run,
        )
        update_task(
            client,
            "verifying",
            "操作后拍照完成，等待验收",
            dry_run=args.dry_run,
            photos={
                "waypoints": waypoint_photos,
                "action_before": action_before_path,
                "action_after": action_after_path,
            },
            result={"status": "done", "message": "已执行拉闸并完成所有任务点照片采集"},
        )

    if not args.skip_photos:
        desktop_photo_dir = copy_photo_bundle(photo_run_dir, args.desktop_photo_dir, dry_run=args.dry_run)
        if desktop_photo_dir:
            photo_payload = {
                "waypoints": waypoint_photos,
                "action_before": action_before_path,
                "desktop_dir": desktop_photo_dir,
            }
            if action_after_path:
                photo_payload["action_after"] = action_after_path
            update_task(
                client,
                "photo_saved",
                "任务点照片已保存并复制到桌面",
                dry_run=args.dry_run,
                photos=photo_payload,
            )

    if args.return_first:
        if args.restart_nav_before_return or nav_was_stopped_for_align:
            log("返航前启动导航")
            if not args.dry_run:
                client.nav_start()
                time.sleep(3.0)
        update_task(client, "returning", "下发返航目标：返回第一个航点", dry_run=args.dry_run, nav_goal=waypoints[0])
        send_goal_and_wait(
            client,
            waypoints[0],
            args.nav_timeout,
            args.return_accept_radius,
            args.nav_adjust,
            retries=args.nav_retries,
            retry_delay=args.nav_retry_delay,
            continue_on_fail=args.continue_on_nav_fail,
            dry_run=args.dry_run,
        )

    update_task(client, "done", "闭环任务完成", dry_run=args.dry_run)
    if not args.dry_run:
        try:
            client.speak("任务完成，请验收")
        except Exception:
            pass
    log("闭环任务完成")


if __name__ == "__main__":
    main()
