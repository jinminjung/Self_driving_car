#!/bin/bash
### This bash is going to spawn Traffic Lights into your simulation at origin 
### Inside the traffic stand
green_light_sdf=$HOME/auto_ws/src/self_driving_car_pkg/models/light_green/model.sdf
red_light_sdf=$HOME/auto_ws/src/self_driving_car_pkg/models/light_red/model.sdf
yellow_light_sdf=$HOME/auto_ws/src/self_driving_car_pkg/models/light_yellow/model.sdf


ros2 run self_driving_car_pkg spawner_node $green_light_sdf green_Light 
sleep 9
ros2 run self_driving_car_pkg spawner_node $yellow_light_sdf yellow_Light 
sleep 2
ros2 run self_driving_car_pkg spawner_node $red_light_sdf red_Light 
