import argparse
import threading
import sys
import socket
import os
import numpy as np
import cv2
from datetime import datetime
import matplotlib.pyplot as plt
import time
from pal.products.qcar import QCar
from pal.utilities.vision import Camera2D
from lidar2 import Lidar
import payload

# Environment Setup
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'

# Car and Camera setup
myCar = QCar(readMode=0)
camera_front = Camera2D(cameraId="3", frameWidth=420, frameHeight=220, frameRate=30)
max_throttle = 0.075
min_throttle = -0.075
max_steering = 0.5
min_steering = -0.5
LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])

# Control variables
steering = 0
throttle = 0
reverse = False
PORT = 38821
stop_threads = False

# LiDAR function to collect and log data
def lidar_func():
    global global_count, stop_threads
    lidar_device = Lidar(type='RPLidar')
    plt.ion()
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

    try:
        while not stop_threads:
            plt.cla()
            lidar_device.read()

            if lidar_device.distances is not None and lidar_device.angles is not None:
                ax.scatter(lidar_device.angles, lidar_device.distances, marker='.')
                ax.set_theta_zero_location("W")
                ax.set_theta_direction(-1)
                ax.set_title("LiDAR Scan Data", va='bottom')
                plt.pause(0.1)
                  
                  
                # Code to convert and collect Lidar Data
                
                # x, y = polar_to_cartesian(lidar_device.angles, lidar_device.distances)
                # data = f"{global_count}, {x.tolist()}, {y.tolist()}\n"
                # catalog.write(data)
                
                global_count += 1
    finally:
        plt.ioff()
        lidar_device.terminate()
        catalog.close()


def avoid_obj(): 
  
    # code logic to avoid objetcts using Lidar
    
    global steering, throttle, max_steering, min_steering, max throttle, min throttle
    
    safe_distance = 0.5
    
    
    while True: 
      
        lidar_device.read()
        if lidar_device.distances is None or lidar_devices.angles is None:
            continue 
            
        
        front_dist = [d for a, d in zip(lidar_device.angles, lidar_device.distances) if -30 <= a <= 30]
        left_dist = [d for a, d in zip(lidar_device.angles, lidar_device.distances) if 30 < a <= 150]
        right_dist = [d for a, d in zip(lidar_device.angles, lidar_device.distances) if -150 <= a < -30]
    
        min_front = min(front_distances, default=float('inf'))
        min_left = min(left_distances, default=float('inf'))
        min_right = min(right_distances, default=float('inf'))
        
        
        if min_front < SAFETY_DISTANCE:
            if min_left > min_right:
                # Turn left to avoid
                steering = max_steering
            else:
                # Turn right to avoid
                steering = min_steering
            throttle = min_throttle  # Slow down or reverse
        elif min_left < SAFETY_DISTANCE:
            steering = min_steering / 2  # Steer slightly right
            throttle = max_throttle / 2  # Slow forward
        elif min_right < SAFETY_DISTANCE:
            steering = max_steering / 2  # Steer slightly left
            throttle = max_throttle / 2  # Slow forward
        else:
            # No obstacles nearby, go straight
            steering = 0
            throttle = max_throttle

        # Apply control
        myCar.throttle_steer(throttle, steering)
        time.sleep(0.1)  # Adjust frequency to match your LiDAR update rate

# Main function to start the threads
def main():
    while True: 
        avoid_obj()
      
main()

  
        
        
        