# Self_driving_car

## Requirement

opencv-contrib-python==3.4.15.55

pygame

## Drive

![스크린샷 2024-11-29 222855](https://github.com/user-attachments/assets/6817c05a-42d8-488a-a70a-5d7c01cedbaf)

```
ros2 launch self_driving_car_pkg light.launch.py

ros2 run self_driving_car_pkg computer_vision_node
```

## Navigation

![스크린샷 2024-12-13 015028](https://github.com/user-attachments/assets/e695ff47-1894-4021-aca0-0435ef19bed5)

```
ros2 launch self_driving_car_pkg navigation.launch.py

ros2 run self_driving_car_pkg sdc_V2
```

## Reference

https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV?tab=readme-ov-file
