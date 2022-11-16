#!/usr/bin/env bash

rqt_plot 	/ximu_talus_l/battery/voltage/data \
		/ximu_talus_r/battery/voltage/data \
		/ximu_tibia_l/battery/voltage/data \
		/ximu_tibia_r/battery/voltage/data \
		/ximu_femur_l/battery/voltage/data \
		/ximu_femur_r/battery/voltage/data \
		/ximu_pelvis/battery/voltage/data  \
		/ximu_torso/battery/voltage/data
