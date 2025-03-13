'''
Program that combines both Lidar and Model
'''
# Import libraries
import threading
import sys
import os
import numpy as np
import time
import cv2
from pal.products.qcar import QCar
from lidar2 import Lidar

from pal.utilities.math import *
from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR
from pal.utilities.vision import Camera2D
import tensorflow as tf
import pygame

# Environment Setup
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
myCar = QCar(readMode=0)
lidar_device = Lidar(type='RPLidar')

# CAR PARAMS: set up +- increment and maximum
deceleration_increment = .00001    
throttle_increment = 0.005
steering_increment = 0.005
max_throttle = 0.1
min_throttle = -0.1
max_steering = 0.5
min_steering = -0.5
MINIMUM_THROTTLE_EPSILON = .0000001

# Autonomous mode should automatically be selected until object in front is detected
AUTONOMOUS_MODE = True

# default speed and direction
throttle = 0.0
steering = 0.0

steering_classes = [-0.5, -.4375, -.375, -.3125, -.25, -.1875, -.125, -.0625, 0, .0625, .125, .1875, .25, .3125, .375, .4375, 0.5]

# stop_threads = False 
# obstacle_detected = False

'''
# Original Car setup for Lidar

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
'''

def lidar_avoidance(): 
    """ LiDAR-based obstacle avoidance """
    global throttle, steering, stop_threads
    
    #steering values to track the steering adjustment before and after avoidance
    last_steering_adjustment = 0.0
    original_path_steer =0.0
    avoiding_obstacles = False
    
    print("Lidar initialized...")  

    while not stop_threads: 
        lidar_device.read()

        # Check if LiDAR data is valid
        if lidar_device.distances is None or lidar_device.angles is None or lidar_device.distances.size == 0:
            print("LiDAR data is empty! Skipping iteration.")
            continue  
        
        new_steering = 0.0

        # Define front view (angles between 60� and 120�)
        front_view = (lidar_device.angles >= np.deg2rad(70)) & (lidar_device.angles <= np.deg2rad(110))
        front_distances = lidar_device.distances[np.nonzero(front_view)]
        
        # Define back view (angles between 180 and 0)
        '''back_view = (lidar_device.angles >= np.deg2rad(180)) & (lidar_device.angles <=np.deg2rad(0))
        back_distances = lidar_device.distances[np.nonzero(back_view)]
        '''

        # Danger threshold for obstacle detection
        danger_threshold = 0.6
        #print(f"Front distances: {front_distances} | Danger threshold: {danger_threshold}")
       
        # Check if there's any obstacle within the danger threshold
        if np.any(front_distances[front_distances > 0] < danger_threshold):
            
            # Define left and right views (angles for obstacle detection)
            left_view = (lidar_device.angles >= np.deg2rad(60)) & (lidar_device.angles <= np.deg2rad(90))
            right_view = (lidar_device.angles >= np.deg2rad(90)) & (lidar_device.angles <= np.deg2rad(120))

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
            print("Obstacle detected! Adjusting steering...")
            if left_clearance > right_clearance + 0.1:
                print("Let's turn left")
                new_steering = max_steering  # Steer left
                last_steering_adjustment = 0.5
                throttle = 0.06
            elif right_clearance > left_clearance + 0.1:
                print("let's turn right")
                new_steering = min_steering  # Steer right
                last_steering_adjustment = -0.5
                throttle = 0.06
            else:
                new_steering = 0.0  # Go straight
              
                throttle = 0.07

            '''
            Now time for us to implement switching to the ML model. 
            Look at AI_FrontCam_Demo.py for reference.
            Also need to adjust steering angle. 
            Further refinement.
            -Rich

            '''
        else:
            print("Time to go back")
            print(f"Returning to OG Path: Last Adjustment: {last_steering_adjustment} Current Steering: {steering} " )

            if(last_steering_adjustment %2 == 0):
                new_steering = last_steering_adjustment * -1
            elif(last_steering_adjustment %2 != 0):
                new_steering = last_steering_adjustment * -1
            else:
                new_steering = 0.0
        
        # Smooth steering adjustment
        steering = 0.7 * steering + 0.3 * new_steering
        print(f"Throttle: {throttle}, Steering: {steering}")

        # Apply control commands to the car
        myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)
        time.sleep(0.1)

def loadModel(filename):
    driving_model = tf.lite.Interpreter(model_path=filename)
    driving_model.allocate_tensors()
    return driving_model
        
def main():
    global stop_threads
    drive_thread = threading.Thread(target=lidar_avoidance if avoid_obstacles else drive_straight)
    
    # Loading in the model 
    driving_model = loadModel("Models/Feb25.tflite")
    input_details = driving_model.get_input_details()
    output_details = driving_model.get_output_details()

    # setup screen size
    screen_width = 800
    screen_height = 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("QCar Autonomous Demo")

    # CAR PARAMS: set up +- increment and maximum
    deceleration_increment = .00001
    throttle_increment = 0.005
    steering_increment = 0.005
    max_throttle = 0.1
    min_throttle = -0.1
    max_steering = 0.5
    min_steering = -0.5
    MINIMUM_THROTTLE_EPSILON = .0000001
    AUTONOMOUS_MODE = False

    throttle = 0.0
    steering = 0.0

    # steering classes coorolated to model prediction via "One Hot Encoding"
    steering_classes = [-0.5, -.4375, -.375, -.3125, -.25, -.1875, -.125, -.0625, 0, .0625, .125, .1875, .25, .3125, .375, .4375, 0.5]

    '''
    While Model is running and no object is detected: 
        if object is detected
            Autonomous mode off
            switch to Lidar
            switch back to model at the end    
    
    '''


    running = True
    while running == True:
        if AUTONOMOUS_MODE is True: 
            for event in pygame.event.get():
                #if the window is closed, shut the program down.
                if event.type == pygame.QUIT:
                    running = False
                    print("Shutting Down")
                    exit(0)
        
            throttle = .075 # locked throttle for steer only models 
            image_cap_np = camPreview(camIDs=["front"]) #take image and store as NP array for efficiency 
            height, width = image_cap_np.shape[:2] 
            image_cap_np = image_cap_np[int(height*.5):int(height*.85),:]
            #crop image to ROI by slicing in half to where only the ground is visible.
            image_cap_grayscale = cv2.cvtColor(image_cap_np, cv2.COLOR_BGR2GRAY) # black and white
            image_cap_grayscale = image_cap_grayscale[np.newaxis,:,:, np.newaxis] #should be shape 1,77,420,1
            image_cap_grayscale = image_cap_grayscale.astype('float32') #model takes in data as float32.
            
            #send image to the model
            driving_model.set_tensor(input_details[0]['index'], image_cap_grayscale)
            driving_model.invoke()
            #predictions is the 17 index array of probabilities [x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x] where 0<=x<=1
            predictions = driving_model.get_tensor(output_details[0]['index'])
            
            class_assign = np.argmax(predictions) #find index of max probability using argmax.
            #print(class_assign)
            steering = steering_classes[class_assign] #give steer index to the car for immediate use. 
   
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