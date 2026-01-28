#!/bin/bash

# 定位到工作空间根目录（假设脚本放在 workspace 根目录或 src 同级目录）
SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
WORKSPACE_DIR=$SCRIPT_DIR

# 如果在编译后的 install 目录下有 setup.bash，先 source
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi

echo "Starting single kick test using realkick.xml..."
ros2 launch brain launch.py tree:=realkick.xml
