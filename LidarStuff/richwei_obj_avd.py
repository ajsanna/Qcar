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
lidar_device = Lidar(type='RPLidar')

# Control variables
steering = 0.0
throttle = 0.07  # Ensure movement starts
stop_threads = False
avoid_obstacles = True

def lidar_avoidance(): 
    """ LiDAR-based obstacle avoidance """
    global throttle, steering, stop_threads
    
    print("Lidar initialized...")  

    while not stop_threads: 
        lidar_device.read()

        # Check if LiDAR data is valid
        if lidar_device.distances is None or lidar_device.angles is None or lidar_device.distances.size == 0:
            print("LiDAR data is empty! Skipping iteration.")
            continue  

        # Define front view (angles between 60° and 120°)
        front_view = (lidar_device.angles >= np.deg2rad(60)) & (lidar_device.angles <= np.deg2rad(120))
        front_distances = lidar_device.distances[np.nonzero(front_view)]

        # Danger threshold for obstacle detection
        danger_threshold = 0.7
        print(f"Front distances: {front_distances} | Danger threshold: {danger_threshold}")
       
        # Check if there's any obstacle within the danger threshold
        if np.any(front_distances[front_distances > 0] < danger_threshold):
            print("Obstacle detected! Adjusting steering...")

            # Define left and right views (angles for obstacle detection)
            left_view = (lidar_device.angles >= np.deg2rad(15)) & (lidar_device.angles <= np.deg2rad(90))
            right_view = (lidar_device.angles >= np.deg2rad(270)) & (lidar_device.angles <= np.deg2rad(345))

            # Get distances for left and right views
            left_distances = lidar_device.distances[left_view]
            right_distances = lidar_device.distances[right_view]

            # Remove NaN values
            left_distances = left_distances[~np.isnan(left_distances)]
            right_distances = right_distances[~np.isnan(right_distances)]

            # Compute average distance for left and right
            left_clearance = np.nanmean(left_distances) if left_distances.size > 0 else np.inf
            right_clearance = np.nanmean(right_distances) if right_distances.size > 0 else np.inf

            print(f"Left Clearance: {left_clearance}, Right Clearance: {right_clearance}")

            # Adjust steering based on clearance comparison
            if left_clearance > right_clearance + 0.1:
                new_steering = max_steering  # Steer left
                throttle = 0.06
            elif right_clearance > left_clearance + 0.1:
                new_steering = min_steering  # Steer right
                throttle = 0.06
            else:
                new_steering = 0.0  # Go straight
        else:
            new_steering = 0.0

        # Smooth steering adjustment
        steering = 0.7 * steering + 0.3 * new_steering
        print(f"Throttle: {throttle}, Steering: {steering}")

        # Apply control commands to the car
        myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)
        time.sleep(0.1)

def correctSteering(steering):
    """ Corrects steering values if needed (for testing) """
    if steering != 0:
        print(f"Corrected Steer: {steering}")
        return steering * -1
    return 0

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
        lidar_device.terminate() 
        myCar.terminate()
        sys.exit()

if __name__ == '__main__':
    main()
