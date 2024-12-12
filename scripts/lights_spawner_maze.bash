#!/bin/bash
### This bash is going to spawn Traffic Lights into your simulation at origin 
### Inside the traffic stand
green_light_sdf=$HOME/.gazebo/models/light_green/model.sdf
red_light_sdf=$HOME/.gazebo/models/light_red/model.sdf
yellow_light_sdf=$HOME/.gazebo/models/light_yellow/model.sdf


ros2 run self_driving_car_pkg spawner_node $red_light_sdf red_light 14.72 4.26 
sleep 7.5
ros2 run self_driving_car_pkg spawner_node $yellow_light_sdf yellow_light 13.7 4.26
sleep 1
ros2 run self_driving_car_pkg spawner_node $green_light_sdf green_Light 12.66 4.26
