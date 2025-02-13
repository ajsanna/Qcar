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
throttle = 0.0  # Ensure movement starts
stop_threads = False
avoid_obstacles = True

'''
Algorithm for LiDar object avoidance

Richwei Chea, Joseph Bui, Sebastian Cursaro

Constant throttle for right now
When an object is detected within the left and/or right angle scopes
the car should turn to the opposite direction of the detected object

STILL IN DEVELOPMENT

'''
def lidar_avoidance(): 
    """ LiDAR-based obstacle avoidance """
    global throttle, steering, stop_threads
    
    print("Lidar initialized...")  

    while not stop_threads: 
        print("\n--- Loop Start ---")  
        lidar_device.read()

        if lidar_device.distances is None or lidar_device.angles is None or lidar_device.distances.size == 0:
            print("LiDAR data is empty! Skipping iteration.")
            continue  

        front_view = (lidar_device.angles >= 345) | (lidar_device.angles <= 15)
        front_distances = lidar_device.distances[front_view]

        '''commenting out for debugging - Bass'''
        #print(f"Front distances: {front_distances}")  

        print("Angles:", lidar_device.angles)


        danger_threshold = 0.25
        if np.any(front_distances < danger_threshold):
            print("Obstacle detected! Adjusting steering...")

            
            # Shouldn't this be 0-90 for right and 90-180 for left side?
            left_view = (lidar_device.angles >= np.deg2rad(15)) & (lidar_device.angles <= np.deg2rad(90))
            right_view = (lidar_device.angles >= np.deg2rad(270)) & (lidar_device.angles <= np.deg2rad(345))
            
            #left_view = (lidar_device.angles >= np.deg2rad(0)) & (lidar_device.angles < np.deg2rad(90))
            #right_view = (lidar_device.angles > np.deg2rad(90)) & (lidar_device.angles <= np.deg2rad(180))

            #added for Bass debugging
            #Stores boolean values if object is detected. Notice how some are FALSE and some are TRUE
            #It is where the specified radians are 15-90 and 270-345, those return TRUE
            
            
            #print("Left view indices: \n", left_view)
            #print("Right view indices: \n", right_view)


            # Filter out NaN values and check if there are valid distances
            left_distances = lidar_device.distances[left_view]
            right_distances = lidar_device.distances[right_view]

            # Exclude NaN values from the distances
            left_distances = left_distances[~np.isnan(left_distances)]
            right_distances = right_distances[~np.isnan(right_distances)]

            # Compute mean only if there are valid distances
            left_clearance = np.nanmean(left_distances) if left_distances.size > 0 else np.inf
            right_clearance = np.nanmean(right_distances) if right_distances.size > 0 else np.inf

            print(f"Left Clearance: {left_clearance}, Right Clearance: {right_clearance}")


            if left_clearance > right_clearance + 0.1:
                new_steering = max_steering * 1 # Steer left
                if np.any(front_distances < danger_threshold):
                    new_steering = correctSteering(new_steering)
                    print(f"New Sigma:  {new_steering}")
            elif right_clearance > left_clearance + 0.1:
                new_steering = min_steering * 1 # Steer right.
                if np.any(front_distances < danger_threshold):
                    new_steering = correctSteering(new_steering)
                    print(f"New Sigma:  {new_steering}")
                    
            else:
                new_steering = 0.0 # Go straight       
            
        else:
            new_steering = 0.0
            
    
        # ? **Ensure steering update happens every loop**
        steering = 0.7 * steering + 0.3 * new_steering
        print(f"Throttle: {throttle}, Steering: {steering}")  

        # ? **Ensure car writes on every loop iteration**
        myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)
        time.sleep(0.1)
        
        
        
        
        
def  correctSteering(steering):
    if(steering > 0):
        print(f"Corrected Steer: {steering}" )
        return steering * -1
    elif(steering < 0):
        print(f"Corrected Steer: {steering}" )
        return steering * -1
    else:
        print(f"Corrected Steer: {steering}" )
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
