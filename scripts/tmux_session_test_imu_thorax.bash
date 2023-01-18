#!/usr/bin/env bash
set -e

tmux new-session -s imu_session -d 
tmux set-option -s -t imu_session default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t imu_session:0.0 "roscore" C-m
sleep 2

tmux new-window
tmux select-window -t 1
tmux split-window -h 
tmux split-window -h 
tmux split-window -h 
tmux select-layout even-horizontal
tmux select-pane -t 3
tmux split-window -v -p 50 
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux select-pane -t 1 
tmux split-window -v -p 50 
tmux select-pane -t 0
tmux split-window -v -p 50 
#tmux select-layout tiled
#tmux select-pane -t 0

#sends keys to first and second terminals
tmux send -t imu_session:1.0 "rviz -d /catkin_ws/src/ximu3_ros/torso_imu_test.rviz" C-m
 #rviz -d ./_default.rviz
tmux send -t imu_session:1.1 "roslaunch ximu3_ros ximu.launch ip_address:=192.168.1.11 receive_port:=9011 send_port:=8011" C-m
#tmux send -t imu_session:1.2 "roslaunch ar_test usb_cal.launch" C-m
#tmux send -t imu_session:1.3 "rostopic echo /inverse_kinematics_from_file/r_data2" C-m
#tmux send -t imu_session:1.4 "rosservice call /inverse_kinematics_from_file/start" C-m
#tmux send -t imu_session:1.5 "roslaunch opensimrt	id.launch" C-m

#tmux send -t imu_session:1.6 "ls -la" C-m
#tmux send -t imu_session:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t imu_session
