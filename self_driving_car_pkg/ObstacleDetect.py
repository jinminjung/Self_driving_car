#!/usr/bin/env python3

import threading


class LidarDetection:

    def __init__(self):
        print("Initialized Object of Obstacle Detection class")

    mode = "Detection"
    detected_class = "Unknown"
    set_time1 = 0
    set_time2 = 0

lidar_det = LidarDetection()

def reset_time1():
    lidar_det.set_time1 = 0
def reset_time2():
    lidar_det.set_time2 = 0

def set_mode(range, Det_class):
    if (range < 11 and Det_class != "Reposition" and lidar_det.set_time1 < 2):
        if (lidar_det.mode == "Detection"):
            lidar_det.mode = "Tracking"
            lidar_det.detected_class = "Avoidance"
            lidar_det.set_time1 += 1
        else:
            lidar_det.mode = "Detection"
            lidar_det.set_time1 += 1
            timer = threading.Timer(100, reset_time1)
            timer.start()


    elif (Det_class == "Reposition" and lidar_det.set_time2 < 2):
        if (lidar_det.mode == "Detection"):
            lidar_det.mode = "Tracking"
            lidar_det.detected_class = "Reposition"
            lidar_det.set_time2 += 1
        else:
            lidar_det.mode = "Detection"
            lidar_det.set_time2 += 1
            timer = threading.Timer(100, reset_time2)
            timer.start()
    else:
        lidar_det.detected_class = Det_class
 
    return lidar_det.mode, lidar_det.detected_class
    