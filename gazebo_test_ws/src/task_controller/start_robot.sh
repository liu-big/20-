#!/bin/bash

# 检查 ROS 环境是否设置
if [ -z "$ROS_MASTER_URI" ]; then
    echo "ROS 环境未设置。请先 source 您的 setup.bash 文件。"
    echo "例如: source ~/catkin_ws/devel/setup.bash"
    exit 1
fi

# 使用 gnome-terminal 打开新选项卡并执行命令
# --working-directory 选项可以设置每个选项卡的工作目录

gnome-terminal --tab --title="Gazebo World" --working-directory="$HOME" -- bash -c "echo '正在启动 Gazebo race world...'; roslaunch gazebo_pkg race.launch; exec bash" \
--tab --title="Random Model Spawner" --working-directory="$HOME/ucar_ws/gazebo_test_ws/src/gazebo_pkg/script" -- bash -c "echo '等待几秒钟让 Gazebo 完全加载...'; sleep 5; echo '正在运行 random_model_for_room.py...'; python3 random_model_for_room.py; exec bash" \
--tab --title="Navigation" --working-directory="$HOME" -- bash -c "echo '等待 Gazebo 和模型加载...'; sleep 10; echo '正在启动导航...'; roslaunch gazebo_nav gazebo_nav.launch; exec bash" \
--tab --title="Task Controller" --working-directory="$HOME" -- bash -c "echo '等待导航启动...'; sleep 5; echo '正在启动任务控制器...'; roslaunch task_controller start_task.launch; exec bash"
