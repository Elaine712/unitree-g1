#!/usr/bin/env bash
set -e

export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR=/home/unitree/g1_dev/yolo11/ultralytics_config
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

source /opt/ros/noetic/setup.bash
source /home/unitree/g1_dev/yolo11/venv/bin/activate

exec python /home/unitree/g1_dev/yolo11/scripts/yolo11_rgb_node.py \
  _image_topic:=/camera/color/image_raw \
  _model_path:=/home/unitree/g1_dev/yolo11/models/switch_yolo11n_relabel_v1.pt \
  _target_hz:=3.0 \
  _imgsz:=640 \
  _conf:=0.40 \
  _publish_debug:=false
