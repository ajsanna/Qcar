import threading
import sys
import os
import numpy as np
import time
from pal.products.qcar import QCar
from lidar2 import Lidar

# Environment Setup
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'

# Car setup
myCar = QCar(readMode=0)
max_throttle = 0.075
min_throttle = -0.075
max_steering = 0.5
min_steering = -0.5
LEDs = np.array([0] * 8)

# Control variables
steering = 0.0
throttle = 0.07  # Ensure movement starts
stop_threads = False
avoid_obstacles = True

def lidar_avoidance(): 
    """ LiDAR-based obstacle avoidance """
    global throttle, steering, stop_threads
    lidar_device = Lidar(type='RPLidar')
    print("Lidar initialized...")  

    while not stop_threads: 
        print("\n--- Loop Start ---")  
        lidar_device.read()

        if lidar_device.distances is None or lidar_device.angles is None or lidar_device.distances.size == 0:
            print("LiDAR data is empty! Skipping iteration.")
            continue  

        front_view = (lidar_device.angles >= 345) | (lidar_device.angles <= 15)
        front_distances = lidar_device.distances[front_view]

        print(f"Front distances: {front_distances}")  

        danger_threshold = 0.25
        if np.any(front_distances < danger_threshold):
            print("Obstacle detected! Adjusting steering...")

            left_view = (lidar_device.angles >= 15) & (lidar_device.angles <= 90)
            right_view = (lidar_device.angles >= 270) & (lidar_device.angles <= 345)

            left_clearance = np.nanmean(lidar_device.distances[left_view]) if left_view.size > 0 else np.inf
            right_clearance = np.nanmean(lidar_device.distances[right_view]) if right_view.size > 0 else np.inf

            print(f"Left Clearance: {left_clearance}, Right Clearance: {right_clearance}")

            if left_clearance > right_clearance + 0.05:
                new_steering = max_steering * 0.5
            elif right_clearance > left_clearance + 0.05:
                new_steering = min_steering * 0.5
            else:
                new_steering = 0.0
        else:
            new_steering = 0.0

        # ? **Ensure steering update happens every loop**
        steering = 0.7 * steering + 0.3 * new_steering
        print(f"Throttle: {throttle}, Steering: {steering}")  

        # ? **Ensure car writes on every loop iteration**
        myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)
        time.sleep(0.1)

def main():
    global stop_threads
    drive_thread = threading.Thread(target=lidar_avoidance if avoid_obstacles else drive_straight)
    
    try:
        drive_thread.start()
        
        while True:
            time.sleep(1)
        
    except KeyboardInterrupt:
        stop_threads = True
    finally:
        myCar.terminate()
        sys.exit()

if __name__ == '__main__':
    main()
