#!/usr/bin/env bash

./temperature_graph.sh &
./battery_charge_graph.sh &
./battery_voltage_graph.sh &
roslaunch ximu3_ros ximu_lower.launch ahrs_divisor:=0
#rqt_plot 	/ximu_talus_l/temperature/data \
#		/ximu_talus_r/temperature/data \
#		/ximu_tibia_l/temperature/data \
#		/ximu_tibia_r/temperature/data \
#		/ximu_femur_l/temperature/data \
#		/ximu_femur_r/temperature/data \
#		/ximu_pelvis/temperature/data  \
#		/ximu_torso/temperature/data
