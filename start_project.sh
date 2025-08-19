#!/bin/bash

# 脚本功能: 使用 tmux 创建一个 2x2 的分屏布局来启动ROS项目。

# 检查工作空间路径是否正确
WORKSPACE_PATH=~/ucar_ws/gazebo_test_ws
if [ ! -f "$WORKSPACE_PATH/devel/setup.bash" ]; then
    echo "错误：找不到ROS工作空间设置文件: $WORKSPACE_PATH/devel/setup.bash"
    exit 1
fi

# 为 tmux 会话创建一个唯一的名字
SESSION_NAME="ros_project_$$"

# 启动一个新的、分离的 tmux 会话
# -d: 分离模式，在后台创建
# -s: 会话名称
# -n: 第一个窗口的名称
tmux new-session -d -s $SESSION_NAME -n "ROS"

# ----------------- 配置四个窗格的命令 -----------------

# 命令1: Gazebo (左上角, 窗格 0)
CMD1="source $WORKSPACE_PATH/devel/setup.bash; echo 'Pane 0: Launching Gazebo...'; roslaunch gazebo_pkg race.launch; exec bash"

# 命令2: Python 脚本 (右上角, 窗格 1)
CMD2="source $WORKSPACE_PATH/devel/setup.bash; cd $WORKSPACE_PATH/src/gazebo_pkg/script; echo 'Pane 1: Waiting 8s...'; sleep 8; python3 random_model_for_room.py; echo 'Python script finished.'; exec bash"

# 命令3: 导航 (左下角, 窗格 2)
CMD3="source $WORKSPACE_PATH/devel/setup.bash; echo 'Pane 2: Waiting 10s...'; sleep 10; roslaunch gazebo_nav gazebo_nav.launch; exec bash"

# 命令4: 任务控制器 (右下角, 窗格 3)
CMD4="source $WORKSPACE_PATH/devel/setup.bash; echo 'Pane 3: Waiting 10s...'; sleep 10; roslaunch task_controller start_task.launch; exec bash"


# ----------------- 创建布局并发送命令 -----------------

# [窗格 0: 左上] 发送命令1
tmux send-keys -t $SESSION_NAME:0.0 "$CMD1" C-m

# 水平分割窗格0，创建 [窗格 1: 右上]
tmux split-window -h -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.1 "$CMD2" C-m

# 选择窗格0 (左上)，然后垂直分割，创建 [窗格 2: 左下]
tmux select-pane -t $SESSION_NAME:0.0
tmux split-window -v
tmux send-keys -t $SESSION_NAME:0.1 "$CMD3" C-m # 新窗格会成为 1, 原来的 1 变成 2

# 选择窗格2 (右上)，然后垂直分割，创建 [窗格 3: 右下]
tmux select-pane -t $SESSION_NAME:0.2
tmux split-window -v
tmux send-keys -t $SESSION_NAME:0.3 "$CMD4" C-m

# 重新调整布局，使所有窗格大小均匀
tmux select-layout -t $SESSION_NAME:0 tiled

# 附加到创建好的 tmux 会话，显示所有窗格
tmux attach-session -t $SESSION_NAME
