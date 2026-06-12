# 宇树 G1 人形机器人综合控制平台

<p align="center">
  <img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ROS Noetic">
  <img src="https://img.shields.io/badge/Python-3.8+-yellow" alt="Python 3.8+">
  <img src="https://img.shields.io/badge/Platform-G1_Humanoid-green" alt="G1 Humanoid">
  <img src="https://img.shields.io/badge/License-MIT-red" alt="License">
</p>

## 📖 项目简介

基于**宇树 G1 人形机器人**的综合控制平台，集成了 2D 导航、灵巧手控制、语音交互、人脸识别、视觉识别等多种功能模块。项目采用 **ROS (Noetic) + Unitree SDK2 Python + Inspire 灵巧手 SDK** 构建，支持通过 WiFi 无线远程控制机器人。

### 🎯 核心能力

- 🤖 **底盘运动控制** — 全向移动、速度可调
- 🦾 **手臂精细控制** — 14 自由度低阶控制 (250Hz)
- ✋ **灵巧手操作** — 双手 12 指独立控制，8 种预设手势
- 🗺️ **2D 自主导航** — SLAM 建图、路径规划、避障
- 🎤 **语音交互** — 语音唤醒、语音导航、语音控制
- 👁️ **视觉识别** — 人脸识别、场景描述
- 🎨 **GUI 控制台** — PyQt5/tkinter 图形界面

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           用户交互层                                      │
├─────────────────────────────────────────────────────────────────────────┤
│   PC 端 GUI                      │        Web 控制台                     │
│   (g1_nav_panel/main.py)         │     (g1_robot_service.py 内置)        │
│   PyQt5 图形界面                  │        浏览器访问                       │
└────────────────┬─────────────────┴───────────────────┬─────────────────┘
                 │           HTTP REST API              │
                 │          (WiFi 无线通信)              │
                 ▼                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        G1 本体控制服务                                     │
│                    g1_robot_service.py (系统核心)                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │
│   │  底盘运动    │  │  手臂控制    │  │  灵巧手     │  │  语音/LED    │  │
│   │  LocoClient │  │  arm_sdk    │  │  Inspire    │  │  AudioClient│  │
│   │  DDS 200Hz  │  │  DDS 250Hz  │  │  Modbus TCP │  │  TTS + LED  │  │
│   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │
│          │                │                │                │          │
└──────────┼────────────────┼────────────────┼────────────────┼──────────┘
           │                │                │                │
           ▼                ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          硬件抽象层                                       │
├─────────────────────────────────────────────────────────────────────────┤
│   宇树 G1 机器人                 │      Inspire RH56E2 灵巧手            │
│   35 个关节 (腿+腰+手臂)          │      双手 12 指 (6指×2)               │
└─────────────────────────────────┴───────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          导航感知层                                       │
├─────────────────────────────────────────────────────────────────────────┤
│   Livox Mid-360 激光雷达                                               │
│          │                                                              │
│          ▼                                                              │
│   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │
│   │  FAST-LIO2  │  │  move_base  │  │  ICP 重定位  │  │  多点导航    │  │
│   │  SLAM 建图   │  │  路径规划    │  │  点云匹配    │  │  示教回放    │  │
│   └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          AI 服务层                                       │
├─────────────────────────────────────────────────────────────────────────┤
│   百度云 API                 │           火山引擎 API                    │
│   人脸识别                    │           视觉大模型                      │
│   (face/face.py)             │           (doubao/test.py)               │
└──────────────────────────────┴─────────────────────────────────────────┘
```

---

## 📁 项目结构

```
HongTu/
├── 📄 核心控制文件 (根目录)
│   ├── g1_robot_service.py          # G1 本体 HTTP 服务 (系统核心)
│   ├── g1_remote_client.py          # HTTP 远程控制客户端
│   ├── g1_control_panel.py          # tkinter 综合控制面板
│   ├── g1_nav_panel/main.py         # PyQt5 导航控制台 (主界面)
│   ├── g1_arm_hand_coordinator.py   # 手臂+灵巧手协同控制
│   ├── demo_press_button.py         # 实验室按钮按压演示脚本
│   ├── g1_xr_motion_test.py         # 手臂运动模式测试
│   ├── hand_control_panel.py        # 灵巧手独立控制面板
│   ├── inspire_hand_driver.py       # 灵巧手 Modbus 驱动
│   ├── start_pc_remote_gui.sh       # PC 本地 GUI + G1 后台一键启动
│   ├── start_g1_backend.sh          # G1 本体 HTTP 后台服务
│   └── setup_hand_relay.sh          # 灵巧手 WiFi 端口转发（按需启动）
│
├── 📂 G1Nav2D/                      # ROS 导航工作空间
│   ├── client/                      # HTTP 客户端 SDK
│   │   ├── DogControllerSDK.py      # 四足机器人 HTTP SDK
│   │   ├── YgClient.py              # cmd_vel 桥接
│   │   └── constants.py             # 运动常量定义
│   ├── src/                         # ROS 功能包
│   │   ├── fastlio2/                # FAST-LIO SLAM 定位
│   │   ├── livox_ros_driver2/       # Livox 激光雷达驱动
│   │   ├── movebase/                # move_base 导航
│   │   ├── pointcloud_to_laserscan/ # 点云转 2D 激光
│   │   ├── ros_map_edit/            # 地图编辑工具
│   │   ├── tool/                    # 示教路径工具
│   │   └── velocity_smoother_ema/   # 速度平滑器
│   └── maps/                        # 地图文件
│
├── 📂 inspire_hand/                 # 灵巧手 SDK
│   └── inspire_sdkpy/
│       ├── inspire_sdk.py           # 单手 Modbus 驱动
│       ├── inspire_sdk_double.py    # 双手 Modbus 驱动
│       ├── inspire_hand_defaut.py   # 默认数据定义
│       └── inspire_dds/             # DDS 消息定义
│
├── 📂 PythonProject/                # 辅助功能子项目
│   ├── daohang/                     # 导航触发脚本
│   ├── doubao/                      # 豆包视觉识别
│   ├── face/                        # 百度云人脸识别
│   ├── point_nav/                   # 单点导航脚本
│   └── py-xiaozhi-main/             # 小智 AI 语音助手
│
└── 📂 unitree_sdk2_python/          # 宇树 SDK2 Python 库
```

---

## 🔧 技术栈详解

### 1. DDS (Data Distribution Service) — 机器人本体通信

**用途**: 机器人内部实时通信（底盘、手臂、音频）

**核心特性**:
| 特性 | 说明 |
|------|------|
| 实时性 | 微秒级延迟，满足机器人控制需求 |
| 可靠性 | 支持可靠传输，数据不丢失 |
| 去中心化 | 无需 broker，节点直接通信 |
| 标准化 | OMG 国际标准，跨平台兼容 |

**DDS 通道列表**:
```
rt/loco          → 底盘运动控制
rt/arm_sdk       → 手臂低阶控制 (14DOF)
rt/arm_action    → 手臂预设动作
rt/audio/api     → 语音/LED 控制
rt/inspire_hand  → 灵巧手状态
```

### 2. Modbus TCP — 灵巧手通信

**用途**: Inspire RH56E2 灵巧手控制

**通信模型**:
```
┌─────────────┐        Modbus TCP         ┌─────────────┐
│   Client     │ ◄───── 请求/响应 ─────► │   Server     │
│ (主站/客户端) │        端口 6000          │ (从站/服务端) │
└─────────────┘                           └─────────────┘
```

**寄存器映射**:
```
0x0000-0x0005: 6 指角度指令 (0-1000)
0x0010-0x0015: 6 指力传感器反馈
0x0020-0x0025: 6 指速度设置
0x0030-0x0035: 6 指力矩限制
```

### 3. ROS (Noetic) — 导航栈

**用途**: 2D 导航、传感器驱动、路径规划

**核心节点**:
| 节点 | 功能 |
|------|------|
| `fastlio2` | SLAM 建图与定位 |
| `livox_ros_driver2` | Livox 激光雷达驱动 |
| `move_base` | 路径规划与避障 |
| `pointcloud_to_laserscan` | 点云转 2D 激光 |
| `slam_reloc` | ICP 重定位服务 |

**核心 Topic**:
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/scan` | LaserScan | 2D 激光数据 |
| `/slam_odom` | Odometry | SLAM 里程计 |
| `/cmd_vel` | Twist | 速度指令 |
| `/move_base/goal` | MoveBaseActionGoal | 导航目标 |

### 4. HTTP REST API — 无线通信

**用途**: PC 端与 G1 本体无线通信

**API 端点**:
```python
# 状态
GET  /api/status          # 查询机器人状态
POST /api/connect         # 连接机器人

# 运动
POST /api/move            # 运动控制 {vx, vy, yaw_rate}
POST /api/stop            # 停止运动

# 模式
POST /api/stand           # 站立
POST /api/fsm             # FSM 模式切换

# 动作
POST /api/action          # 预设动作
POST /api/coordinated     # 协同动作

# 语音/LED
POST /api/speak           # TTS 语音播报
POST /api/volume          # 音量调节
POST /api/led             # LED 颜色

# 灵巧手
POST /api/hand/preset     # 手势预设
POST /api/hand/angles     # 自定义角度

# 手臂
POST /api/arm/activate    # 激活 arm_sdk
POST /api/arm/release     # 释放 arm_sdk
POST /api/arm/joints      # 关节角度控制

# 导航
POST /api/nav/start       # 启动导航栈
POST /api/nav/stop        # 停止导航栈
POST /api/nav/goal        # 发送导航目标
POST /api/nav/reloc       # ICP 重定位

# 任务闭环
GET  /api/task            # 查询任务状态机
POST /api/task/update     # 更新任务阶段/识别/操作/验收状态
POST /api/task/reset      # 重置任务状态机
```

### 5. FAST-LIO2 — SLAM 建图与定位

**用途**: 实时建图与定位

**算法特点**:
- 紧耦合 IMU + 激光雷达数据融合
- 迭代卡尔曼滤波 (iKF) 状态估计
- 增量式点云地图构建
- 对退化场景（长走廊等）鲁棒

### 6. move_base — 路径规划与避障

**用途**: 全局路径规划 + 局部避障

**两级规划**:
| 层级 | 算法 | 作用 |
|------|------|------|
| 全局规划 | Dijkstra / A* | 在静态地图上规划最优路径 |
| 局部规划 | DWA | 实时避障，跟踪全局路径 |

### 7. ICP — 点云匹配重定位

**用途**: 机器人初始定位（不知道自己在哪里时）

**重定位流程**:
```
用户在 RViz 指定初始位姿 → /initialpose 话题
    → slam_reloc.py 转换
    → 调用 /slam_reloc 服务
    → FAST-LIO 执行 ICP 点云匹配
    → 更新机器人位姿
```

### 8. 百度云 API — 人脸识别

**用途**: 识别人员身份

**流程**:
```
摄像头拍照 → Base64 编码 → 百度云 API → 返回人员身份
```

### 9. 火山引擎 API — 视觉大模型

**用途**: 场景描述、目标识别

**流程**:
```
摄像头拍照 → 火山引擎豆包视觉大模型 → 返回场景描述
```

### 10. MQTT — 语音助手通信

**用途**: 小智 AI 语音助手设备通信

**特性**:
- 轻量级协议，适合物联网
- 发布-订阅模式
- 支持 QoS 服务质量

### 11. PyQt5 / tkinter — GUI 界面

**用途**: 图形化控制界面

**对比**:
| 特性 | PyQt5 | tkinter |
|------|-------|---------|
| 功能 | 丰富、专业 | 基础、够用 |
| 外观 | 现代、美观 | 简朴 |
| 依赖 | 需要安装 | Python 自带 |

---

## 🚀 部署指南

### 环境要求

- **操作系统**: Ubuntu 20.04 (推荐)
- **ROS 版本**: ROS Noetic
- **Python 版本**: 3.8+
- **硬件**: 宇树 G1 人形机器人、Livox Mid-360 激光雷达、Inspire RH56E2 灵巧手

### 克隆仓库

```bash
git clone https://github.com/yuanqizhiti/HongTu.git
cd HongTu
```

### 2D 导航部署

#### 1. 安装 Livox SDK2

```bash
sudo apt install cmake
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```

#### 2. 配置雷达和地图路径

```bash
# 修改本机与雷达 IP
cd G1Nav2D/src/livox_ros_driver2-master/config/
gedit MID360_config.json

# 修改地图保存路径
cd G1Nav2D/src/fastlio2/src/
gedit map_builder_node.cpp
```

#### 3. 编译程序

```bash
cd G1Nav2D/
catkin_make

# 遇到报错可先执行
cd src/livox_ros_driver2-master/
./build.sh ROS1
cd ../../
catkin_make
```

#### 4. 安装依赖包

```bash
sudo apt install ros-noetic-teb-local-planner \
                 ros-noetic-global-planner \
                 ros-noetic-costmap-server
```

#### 5. 建图与保存

```bash
# 启动建图
source devel/setup.bash
roslaunch fastlio mapping.launch

# 新终端保存地图
source devel/setup.bash
rosrun map_server map_saver map:=/projected_map -f ~/mymap
```

#### 6. 编辑地图

```bash
# 使用 Map Eraser Tool 清除动态障碍物
source devel/setup.bash
roslaunch ros_map_edit map_edit.launch

# 可选：直接用图片编辑器查看/微调 PGM 地图
gimp ~/Desktop/G1map.pgm
```

#### 7. 启动导航

```bash
# 修改地图路径
gedit src/fastlio2/config/gridmap_load.launch

# 可选：手动调用 ICP 重定位；GUI/RViz 中常用 2D Pose Estimate 完成同类操作
source ~/Desktop/HongTu/G1Nav2D/devel/setup.bash
rosservice call /slam_reloc "{pcd_path: '/home/zgx/Desktop/HongTu/G1Nav2D/src/fastlio2/PCD/map.pcd', x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"

# 启动导航
source devel/setup.bash
roslaunch fastlio navigation.launch
```

### 语音交互部署

基于 [py-xiaozhi](https://github.com/huangjunsen0406/py-xiaozhi) 项目。

#### 环境要求

- Python 3.9 - 3.12
- 麦克风和扬声器
- 稳定的互联网连接

#### 安装依赖

```bash
cd PythonProject/py-xiaozhi-main/
pip install -r requirements.txt
```

#### 配置语音导航

1. **修改关键词**: 全局搜索 "电梯"，替换为你的目标关键词（如 "卧室"、"卫生间"）

2. **修改目标坐标**: 在 `PythonProject/point_nav/point2.py` 底部修改坐标
   - 坐标可通过监听 `/move_base/goal` 话题获取

3. **配置 MCP 服务**:
   ```bash
   # 添加导航关键词
   PythonProject/py-xiaozhi-main/src/application.py  # 第 1232 行

   # 注册 MCP 服务
   PythonProject/py-xiaozhi-main/src/mcp/mcp_server.py  # 第 334 行

   # 配置启动脚本
   PythonProject/py-xiaozhi-main/src/mcp/tools/daohang_dianti/tools.py  # 第 15-17 行
   PythonProject/daohang/daohang-dianti.py  # 第 10-13 行

   # 修改目标坐标
   PythonProject/point_nav/point1.py
   ```

#### 启动语音程序

```bash
cd PythonProject/py-xiaozhi-main/
python3 main.py
```

> ⚠️ **注意**: 实现语音导航需要同时开启语音、运控、导航三个程序。

### G1 本体部署

#### 方案一：本体运行 GUI（推荐无线方案）

GUI 和控制进程部署到 G1 本体，PC 通过 SSH X11/VNC/NoMachine 远程显示。

```bash
# PC 有线连接 G1 后部署
./deploy_to_g1.sh

# 远程启动 GUI
./remote_robot_gui.sh
```

> 💡 脚本会自动尝试有线 `192.168.123.164` 和无线 `192.168.13.24`

手动指定：
```bash
G1_HOST=192.168.13.24 ./remote_robot_gui.sh
```

#### 方案二：PC 本地 GUI + G1 后台服务

GUI 在 PC 本地运行，G1 只运行 HTTP 后台服务。

```bash
# 部署到 G1
./deploy_to_g1.sh

# PC 本地一键启动 GUI，并自动探测/拉起 G1 后台
./start_pc_remote_gui.sh
```

启动优先级：

```text
上次成功 IP → 常用 WiFi IP → 有线 IP → 当前 WiFi 网段自动探测
```

常用 IP：

```text
有线: 192.168.123.164
无线: 192.168.13.24
```

手动指定：

```bash
G1_BACKEND_HOST=192.168.13.24 ./start_pc_remote_gui.sh
G1_BACKEND_HOST=192.168.123.164 ./start_pc_remote_gui.sh
```

G1 本体后台也可以手动启动：

```bash
ssh unitree@192.168.123.164
cd /home/unitree/zgx_g1
./start_g1_backend.sh
```

Web 控制台：
```
http://192.168.13.24:5055/
```

> 说明：当前不在 G1 本体设置 HongTu 自启动项。`start_g1_backend.sh` 只在你运行 `start_pc_remote_gui.sh` 或手动启动后台时执行。灵巧手端口转发也只会随后台启动时尝试配置一次，不再通过 crontab 开机自启动。

Web 控制台包含：

- 导航控制：启动/停止导航、地图点选、航点巡航、遥控。
- 姿态设置：灵巧手、arm_sdk 低阶臂控、后台状态。
- 任务闭环：报警、导航、识别、操作、验收、返航等状态记录。

任务状态会保存到 G1 本体：

```text
/home/unitree/zgx_g1/.runtime/task_state.json
```

当前任务闭环只负责状态记录和结果预览，拍照窗口为预留位；后续接入相机后再写入操作前/后图片路径和 `result.json`。

---

## 🎮 功能使用指南

### 1. 底盘运动控制

**GUI 操作**:
- 使用 WASD 控制前后左右
- 使用 QE 控制旋转
- 速度滑块调节运动速度

**API 调用**:
```python
from g1_remote_client import G1RemoteClient

client = G1RemoteClient("http://192.168.13.24:5055")
client.connect()

# 前进
client.move(vx=0.3, vy=0.0, yaw_rate=0.0)

# 停止
client.stop()
```

### 2. 手臂控制

**预设动作**:
```python
# 执行预设动作
client.action("wave")       # 挥手
client.action("point")      # 指向
client.action("open_arms")  # 展开双臂
```

**低阶控制 (arm_sdk)**:
```python
from g1_remote_client import G1RemoteClient

client = G1RemoteClient("http://192.168.13.24:5055")

# 激活 arm_sdk
client.arm_activate()

# 设置双臂 14 个关节角
# 顺序: 左臂 7 个 + 右臂 7 个
client.arm_joints([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
])

# 释放 arm_sdk
client.arm_release()
```

**协同动作**:
```python
# 执行手臂+灵巧手协同动作
client.coordinated("wave_hello")
```

### 3. 灵巧手控制

**预设手势**:
```python
# 右手握拳
client.hand_preset("r", "fist")

# 左手张开
client.hand_preset("l", "open")
```

**自定义角度**:
```python
# 设置右手 6 指角度 (0-1000)
client.hand_angles("r", [500, 300, 800, 700, 600, 500])
```

**预设手势列表**:
| 手势 | 说明 |
|------|------|
| `open` | 张开 |
| `fist` | 握拳 |
| `point` | 指向 |
| `ok` | OK 手势 |
| `thumbs_up` | 点赞 |
| `rock` | 摇滚 |
| `pinch` | 三指捏 |
| `half` | 半开 |

### 4. 2D 导航

**启动导航栈**:
```python
client.nav_start()
```

**发送导航目标**:
```python
# 导航到指定坐标
client.nav_goal(x=2.0, y=3.0, theta=0.0)
```

**ICP 重定位**:
```python
# 当机器人不知道自己在哪里时
client.nav_reloc(x=1.0, y=2.0, theta=0.0)
```

**多点巡航**:
```python
# 从 JSON 加载航点
import json
with open("waypoints.json") as f:
    waypoints = json.load(f)

# 依次导航到每个航点
for wp in waypoints:
    client.nav_goal(wp["x"], wp["y"], wp["theta"])
    # 等待到达
```

### 5. 语音控制

**TTS 播报**:
```python
client.speak("你好，我是 G1 机器人")
```

**音量调节**:
```python
client.volume(80)  # 0-100
```

### 6. LED 控制

```python
# 设置 LED 颜色 (RGB)
client.led(255, 0, 0)    # 红色
client.led(0, 255, 0)    # 绿色
client.led(0, 0, 255)    # 蓝色
```

### 7. 实验室按钮按压演示

演示脚本：

```bash
demo_press_button.py
```

用途：

```text
导航到固定点
→ 视觉/YOLO 确认目标
→ 执行手臂预设姿态
→ 快速执行示教按压动作
→ 按压后安全抬起
→ 回初始姿态
→ 释放 arm_sdk
```

需要在 `~/Desktop/g1_poses.json` 中保存以下动作名，并同步到 G1：

```text
按压平举预设动作
按下动作
按压后抬起动作
初始位置
```

可选动作：

```text
收手过渡态
```

同步姿态文件：

```bash
rsync -az ~/Desktop/g1_poses.json unitree@192.168.13.24:/home/unitree/zgx_g1/g1_poses.json
```

运行示例：

```bash
ssh unitree@192.168.13.24 \
  'cd /home/unitree/zgx_g1 && ./demo_press_button.py --taught-press-duration 0.18 --taught-press-steps 4 --hold-seconds 1'
```

当前默认流程：

```text
启动臂控
→ 收手过渡态
→ 按压平举预设动作
→ 按下动作
→ 保持
→ 按压后抬起动作
→ 初始位置
→ 释放臂控
```

参数说明：

| 参数 | 说明 |
|------|------|
| `--taught-press-duration` | 从按压平举到按下动作的时间，越小越快 |
| `--taught-press-steps` | 示教按压插值步数，越少越硬 |
| `--hold-seconds` | 到达按下动作后保持时间 |
| `--post-press-lift-name` | 按压后安全抬起动作名，默认 `按压后抬起动作` |
| `--skip-transition-after-lift` | 使用安全抬起后是否跳过收手过渡态，默认 1 |

### 8. 工业拉闸演示

演示脚本：

```bash
demo_pull_switch.py
```

用途：

```text
导航到固定点
→ YOLO/深度相机确认拉闸位置
→ 语音播报：检测到电闸未关闭导致漏电，现关闭电闸
→ 执行接近/抓握/下拉/脱离预设动作
→ 拍照验收（预留）
→ 语音播报任务完成
→ 释放 arm_sdk
```

需要在 `g1_poses2.json` 中保存以下动作名：

```text
拉闸接近动作
拉闸抓握动作
拉闸下拉动作
拉闸脱离动作
初始位置
```

运行示例：

```bash
ssh unitree@192.168.13.24 \
  'cd /home/unitree/zgx_g1 && ./demo_pull_switch.py --speak 1'
```

> 当前版本按预设动作执行，不做 IK 在线解算；视觉检测和深度坐标用于确认目标和后续微调。

一键方式（PC 端，有线/无线自动探测）：

```bash
cd ~/Desktop/HongTu
./run_pull_switch_demo.sh --speak 1
```

按压脚本同风格手动方式：

```bash
# 有线
ssh unitree@192.168.123.164 'cd /home/unitree/zgx_g1 && mkdir -p .runtime && nohup ./start_g1_backend.sh > .runtime/g1_backend.log 2>&1 < /dev/null & sleep 6 && ss -ltn | grep 5055 && pgrep -af g1_robot_service.py'
ssh unitree@192.168.123.164 'cd /home/unitree/zgx_g1 && ./demo_pull_switch.py --poses g1_poses2.json --speak 1'

# 无线
ssh unitree@192.168.13.24 'cd /home/unitree/zgx_g1 && mkdir -p .runtime && nohup ./start_g1_backend.sh > .runtime/g1_backend.log 2>&1 < /dev/null & sleep 6 && ss -ltn | grep 5055 && pgrep -af g1_robot_service.py'
ssh unitree@192.168.13.24 'cd /home/unitree/zgx_g1 && ./demo_pull_switch.py --poses g1_poses2.json --speak 1'
```

---

## 🛡️ 安全机制

### 1. 运动安全

| 机制 | 说明 |
|------|------|
| **Watchdog** | 200ms 超时自动停止运动 |
| **速度限制** | 关节运动速度裁剪 |
| **倾角保护** | IMU roll/pitch 超 0.45 rad 自动释放手臂 |

### 2. 手臂安全

| 机制 | 说明 |
|------|------|
| **Weight 渐变** | arm_sdk 激活/释放时约 1 秒渐入渐出 |
| **FSM 前置检查** | 模式切换前自动释放 arm_sdk |
| **Lowstate 检查** | 激活前检查关节状态 |
| **速度限幅** | arm_sdk 目标角按 250Hz 发布并限速 |
| **释放前回安全姿态** | 业务脚本先回安全/初始姿态，再释放 arm_sdk |

#### arm_sdk 释放逻辑

`arm_release()` 本身不是“自动回初始位置”，而是：

```text
arm_sdk weight 平滑降到 0
停止 arm_sdk 发布线程
释放低阶臂控
```

因此安全流程应当是：

```text
先用关节插值回到安全姿态/初始位置
再调用 arm_release()
```

`demo_press_button.py` 当前就是按这个逻辑执行：

```text
按下动作
→ 按压后抬起动作
→ 初始位置
→ arm_release()
```

#### 腰部锁定逻辑

当前 arm_sdk 控制的 14 个关节是：

```text
左臂 7 个: 15-21
右臂 7 个: 22-28
```

腰部关节是：

```text
12 WaistYaw
13 WaistRoll
14 WaistPitch
```

腰部不在 `G1_ARM_JOINT_IDS` 中。后台初始化 LowCmd 时会读取当前全身 `lowstate`，把腰部保持在接管时的位置；后续 `/api/arm/joints` 只更新双臂 14 个关节，不主动改变腰部。

### 3. 导航安全

| 机制 | 说明 |
|------|------|
| **自动暂停** | 导航前自动暂停 `/control` 节点 |
| **避障** | move_base 实时避障 |
| **膨胀区** | 代价地图障碍物膨胀 |

---

## 📊 核心文件说明

| 文件 | 行数 | 功能 |
|------|------|------|
| `g1_robot_service.py` | ~1500 | **系统核心** — G1 本体 HTTP 服务 |
| `g1_nav_panel/main.py` | ~3700 | **主界面** — PyQt5 导航控制台 |
| `g1_control_panel.py` | ~800 | tkinter 综合控制面板 |
| `g1_arm_hand_coordinator.py` | ~500 | 手臂+灵巧手协同控制 |
| `g1_remote_client.py` | ~200 | HTTP 远程客户端 |
| `hand_control_panel.py` | ~600 | 灵巧手独立控制面板 |
| `inspire_hand_driver.py` | ~100 | 灵巧手 Modbus 驱动 |

---

## 🔌 硬件连接

### 网络拓扑

```
┌─────────────┐         WiFi          ┌──────────────┐
│   PC 端      │ ◄──────────────────► │   G1 本体     │
│ 192.168.1.x │                       │ 192.168.13.24│
└─────────────┘                       └──────┬───────┘
                                             │
                              ┌───────────────┼───────────────┐
                              │               │               │
                              ▼               ▼               ▼
                        ┌──────────┐   ┌──────────┐   ┌──────────┐
                        │ 激光雷达  │   │  灵巧手   │   │  摄像头   │
                        │ Livox    │   │ Inspire  │   │  Camera  │
                        │ Mid-360  │   │ RH56E2   │   │          │
                        └──────────┘   └──────────┘   └──────────┘
```

### IP 配置

| 设备 | IP 地址 | 用途 |
|------|---------|------|
| G1 本体（有线） | 192.168.123.164 | SSH/部署/调试 |
| G1 本体（无线） | 192.168.13.24 | PC GUI / Web / HTTP 后台 |
| Livox 雷达 | 192.168.123.120 | 激光雷达 |
| Inspire 左手 | 192.168.123.210:6000 | Modbus TCP |
| Inspire 右手 | 192.168.123.211:6000 | Modbus TCP |

灵巧手 WiFi 端口转发：

```text
G1:5021 → 左手 192.168.123.210:6000
G1:5022 → 右手 192.168.123.211:6000
```

端口转发不再开机自启动，只会在 `start_g1_backend.sh` 启动时尝试配置一次：

```bash
HONGTU_SETUP_HAND_RELAY=1 ./start_g1_backend.sh
```

如需禁用：

```bash
HONGTU_SETUP_HAND_RELAY=0 ./start_g1_backend.sh
```

---

## ⚠️ 已知风险与排查点

### 1. 后台自动拉起失败后可能出现空 URL

`start_pc_remote_gui.sh` 会先 HTTP 探测，再 SSH 到 G1 拉起后台。如果 SSH 可连但后台启动失败，脚本会尝试 SSH 隧道；当前隧道失败时仍可能继续打开 GUI，日志中会出现：

```text
[pc-gui] backend: http://
```

出现该情况时先手动检查本体后台：

```bash
ssh unitree@192.168.13.24
cd /home/unitree/zgx_g1
tail -80 .runtime/g1_backend.log
./start_g1_backend.sh
```

### 2. DDS 网卡名仍依赖本体实际接口

`start_g1_backend.sh` 默认使用：

```bash
HONGTU_G1_NET_IF=eth0
```

如果 G1 本体实际 DDS/雷达内网接口变成 `eth1`，后台会报 `eth0: does not match an available interface`。这时不要改系统网卡名，优先临时指定：

```bash
HONGTU_G1_NET_IF=eth1 ./start_g1_backend.sh
```

### 3. 导航启动会暂停部分原厂/旧导航进程

为了避免多个节点同时发布 `/cmd_vel`、TF 或定位结果，`g1_robot_service.py` 在启动导航时会清理匹配到的导航、定位和 `/control` 进程。若另一台 PC 正在跑旧版导航，同一时间启动本项目后台导航可能互相影响。

建议现场只保留一套导航栈运行；切换电脑前先停止当前导航：

```bash
curl -X POST http://192.168.13.24:5055/api/nav/stop
```

### 4. 灵巧手端口转发会修改本体 iptables

`setup_hand_relay.sh` 是一次性、幂等配置，但会修改：

- `net.ipv4.ip_forward`
- `iptables` NAT/FORWARD 规则

它不再通过 crontab 自启动，只随 `start_g1_backend.sh` 尝试执行。若不需要 WiFi 转发灵巧手，使用：

```bash
HONGTU_SETUP_HAND_RELAY=0 ./start_g1_backend.sh
```

### 5. 按压/拉闸演示仍依赖示教动作质量

`demo_press_button.py` 和 `demo_pull_switch.py` 当前以示教姿态为主。导航误差、目标安装高度、手指接触方向都会影响结果；正式演示前需要确认：

- `g1_poses.json` / `g1_poses2.json` 已按对应任务同步到 G1。
- 按压后的安全抬起动作不会扫到桌面。
- 拉闸动作的“脱离动作”先远离墙面，再回初始位置。
- `arm_sdk` 释放前动作已经回到安全姿态。

---

## 📝 开发说明

### 添加新的预设动作

在 `g1_arm_hand_coordinator.py` 的 `BUILTIN_ACTIONS` 中添加：

```python
BUILTIN_ACTIONS = {
    # ... 已有动作
    "my_custom_action": {
        "description": "自定义动作",
        "frames": [
            {"time": 0.0, "joints": {...}, "hand": {...}},
            {"time": 1.0, "joints": {...}, "hand": {...}},
        ]
    }
}
```

### 添加新的 API 端点

在 `g1_robot_service.py` 的 `Handler.do_POST` 中添加：

```python
elif path == "/api/my_endpoint":
    # 处理请求
    data = json.loads(post_data)
    result = self.robot.my_function(data)
    self.send_json({"ok": True, "result": result})
```

### 添加新的导航点

在 `PythonProject/point_nav/` 中创建新脚本：

```python
#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseActionGoal

rospy.init_node("nav_to_my_point")
pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)

goal = MoveBaseActionGoal()
goal.goal.target_pose.header.frame_id = "map"
goal.goal.target_pose.pose.position.x = 1.0  # X 坐标
goal.goal.target_pose.pose.position.y = 2.0  # Y 坐标
goal.goal.target_pose.pose.orientation.w = 1.0

pub.publish(goal)
```
