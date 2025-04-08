'''
=========================================================================================
Program that combines both Lidar and ML model for autonomous vehicle navigation
=========================================================================================

Description:
------------
This Program allows the Qaunser Q Car to utilize the LiDar readings to 
avoid objects and countersteer back into path,and if no object is detected 
return to using the model to follow the intended path. 
EX: a loop with clear lane markings or a custom path.


LiDar Features:
---------------
1. **Field of View**: We set the front feild of view to track objects from degrees 70 to 110.
                    This reduces the noise from objects that are behind the car or to the side of the car.
2. **Danger Threshold**: Set to .6. Any object detected within this threshold would then trigger the object avoidance logic,
                        allowing the car to steer left or right based off of the clearence detected from each side rspectively.
3. **Counter Steer**: After the car has sucessfully avoided the object, it then would perform a smooth counter steer, to return
                    back to the deviated path.

Model Features:
---------------
1. **Model Loading**:
   - A TensorFlow Lite model is loaded via `loadModel("Models/Feb25.tflite")`.
   - The model expects an input image and returns an array of probabilities for 17 steering classes.
2. **Autonomous Driving**:
   - The program continuously captures images from the front camera.
   - The image is preprocessed (converted to grayscale and cropped).
   - The image is sent to the TFLite model for classification.
   - The predicted steering class is mapped to a steering value.
   - Steering is adjusted based on the models prediction.
   
How To Use:
-----------
1. Place the desired **TFLite model** inside the `Models/` folder.
2. Modify the model path on **line 254** if necessary.
3. Ensure the Qanser Lidar File is in the same Directory. IE "lidarThread.py"
4. Run the program using: py3 LidarAndModel.py
5. EXIT with "ctrl+c"

Authors:
--------
- **Richwei Chea** (richweichea@cpp.edu)
- **Sebastian Cursaro** (scursaro@cpp.edu)
- **Joseph Bui** (jhbui@cpp.edu)

Model Engineers:
--------
- **Alexander Sanna** (ajsanna@cpp.edu)
- **Matthew Baldivino** (mabaldivino@cpp.edu)

'''
# Import libraries
import threading
import sys
import os
import numpy as np
import time
import cv2
from pal.products.qcar import QCar
from lidarThread import Lidar

from pal.utilities.math import *
from pal.utilities.vision import Camera2D
import tensorflow as tf


import pygame

# Environment Setup
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
myCar = QCar(readMode=0)
lidar_device = Lidar(type='RPLidar')

# Initialize cameras - this follows the working approach from AutonomousLaneLoop
camera_front = Camera2D(cameraId="3", frameWidth=420, frameHeight=220, frameRate=30) 

# CAR PARAMS: set up +- increment and maximum
deceleration_increment = 0.00001    
throttle_increment = 0.005
steering_increment = 0.005
max_throttle = 0.1
min_throttle = -0.1
max_steering = 0.5
min_steering = -0.5
MINIMUM_THROTTLE_EPSILON = 0.0000001

# Initialize car control values
throttle = 0.0
steering = 0.0
LEDs = np.array([0] * 8)

# Steering classes for model prediction via "One Hot Encoding"
steering_classes = [-0.5, -0.4375, -0.375, -0.3125, -0.25, -0.1875, -0.125, -0.0625, 0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375, 0.5]

stop_threads = False
obstacle_detected = False

# Load the ML driving model
def loadModel(filename):
    driving_model = tf.lite.Interpreter(model_path=filename)
    driving_model.allocate_tensors()
    return driving_model
    

# Using the camPreview function from AutonomousLaneLoop
def camPreview(camIDs, steering=0, throttle=0, catalog=None):
    if camIDs is not None:
        if "front" in camIDs:
            camera_front.read()
            if camera_front is not None:
                return camera_front.imageData
    return None

# Get steering prediction from ML model
def getModelSteering(model, input_details, output_details):
    """
    Get steering prediction from the ML model
    Returns: steering value
    """
    counter =0

    image_cap_np = camPreview(camIDs=["front"])
    height, width = image_cap_np.shape[:2] 
    image_cap_np = image_cap_np[int(height*.5):int(height*.85),:]
    #crop image to ROI by slicing in half to where only the ground is visible.
    image_cap_grayscale = cv2.cvtColor(image_cap_np, cv2.COLOR_BGR2GRAY) # black and white
    image_cap_grayscale = image_cap_grayscale[np.newaxis,:,:, np.newaxis] #should be shape 1,77,420,1
    image_cap_grayscale = image_cap_grayscale.astype('float32') #model takes in data as float32.
                
    photo = image_cap_grayscale
    height, width = photo.shape[:2]

    offset = int(width * 0.05)  # Adjust this as needed

    # Define top-left and top-right triangular regions with a gap
    top_left_triangle = np.array([[0, 0], [width // 2 - offset, 0], [0, height // 1.5 - offset]], np.int32)
    top_right_triangle = np.array([[width, 0], [width // 2 + offset, 0], [width, height // 1.5 - offset]], np.int32)

    # Create a mask (white image).
    mask = np.ones_like(photo, dtype=np.uint8) * 255
    mask = mask.astype('float32')
                # Draw black triangles over the corners
    cv2.fillPoly(mask, [top_left_triangle, top_right_triangle], (0, 0, 0))
                
                # Apply the mask to remove the corners
    image_cap_grayscale = cv2.bitwise_and(photo, mask)
    #take 10 pixels off the top for further cropping down to 1,67,420,1
    #print(image_cap_grayscale.shape)
    image_cap_grayscale = image_cap_grayscale[:,10:,:,:]
               
    
    # Send image to the model
    model.set_tensor(input_details[0]['index'], image_cap_grayscale)
    model.invoke()
    
    # Get predictions
    predictions = model.get_tensor(output_details[0]['index'])
    
    
    class_assign = np.argmax(predictions)
    
    counter+=1
    print(f"\nPicture Sent: {counter} ")

    return steering_classes[class_assign]

# LiDAR thread to detect obstacles and do avoidance
def lidarThreadFunction(model, input_details, output_details):
    """
    Thread for LiDAR processing and mode switching
    """
    global throttle, steering, stop_threads, obstacle_detected
    
    # Variables for obstacle avoidance
    last_steering_adjustment = 0.0
    original_steering = 0.0
    
    print("LiDAR thread initialized...")
    
    while not stop_threads:

        lidar_device.read()
        
        # Check if LiDAR data is valid
        if lidar_device.distances is None or lidar_device.angles is None or lidar_device.distances.size == 0:
            print("LiDAR data is empty! Skipping iteration.")
            # time.sleep(0.1)
            continue
        
        # Define front view (angles between 70° and 110°)
        front_view = (lidar_device.angles >= np.deg2rad(70)) & (lidar_device.angles <= np.deg2rad(110))
        front_distances = lidar_device.distances[np.nonzero(front_view)]
        
        # Remove invalid values
        front_distances = front_distances[front_distances > 0]

        danger_threshold = 0.6
        
        # Check if there's any obstacle within the danger threshold
        if len(front_distances) > 0 and np.any(front_distances < danger_threshold):
            if not obstacle_detected:
                print("Obstacle detected! Switching to LiDAR avoidance mode")
                obstacle_detected = True
                original_steering = steering  # Save current steering
            
            # Define left and right views
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
            if left_clearance > right_clearance + 0.1:
                print("Steering left to avoid obstacle")
                new_steering = max_steering  # Steer left
                last_steering_adjustment = 0.5
                throttle = 0.07
            elif right_clearance > left_clearance + 0.1:
                print("Steering right to avoid obstacle")
                new_steering = min_steering  # Steer right
                last_steering_adjustment = -0.5
                throttle = 0.07
            else:

                new_steering = 0.0  # Go straight
                throttle = 0.08
            
            # Smooth steering adjustment
            steering = 0.7 * steering + 0.3 * new_steering
            
        else:
            if obstacle_detected:
                print("No obstacle detected! Switching back to ML model navigation")
                obstacle_detected = False
                
                # Optional: Apply counter-steering to return to original path
                if last_steering_adjustment != 0:
                    print(f"Applying correction: Last adjustment: {last_steering_adjustment}")
                    counter_steering = -0.5 * last_steering_adjustment
                    steering = 0.7 * steering + 0.3 * counter_steering
                    last_steering_adjustment = 0.0
                
            else:
                # If no obstacle and already in ML mode, get steering from model
                if not stop_threads:  # Check again to prevent accessing the model after shutdown
                    new_steering = getModelSteering(model, input_details, output_details)
                    # Smooth steering transition
                    steering = 0.8 * steering + 0.3 * new_steering
                    throttle = 0.075  # Default throttle for ML model
        
        # Apply control commands to the car
        myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)
        time.sleep(0.1)

def main():
    global stop_threads, throttle, steering
    
    # Load the ML model
    print("Loading ML model...")
    driving_model = loadModel("Models/Mar12Class.tflite")
    input_details = driving_model.get_input_details()
    output_details = driving_model.get_output_details()
    
    print("ML model loaded successfully")
    
    # Start the LiDAR thread
    lidar_thread = threading.Thread(
        target=lidarThreadFunction, 
        args=(driving_model, input_details, output_details)
    )
    lidar_thread.daemon = True
    lidar_thread.start()
    
    try:
        print("Main control loop started. Press Ctrl+C to exit.")
        running = True
        while running:
            # Print status to console
            mode_text = "LiDAR Avoidance" if obstacle_detected else "ML Navigation"
            print(f"Mode: {mode_text}, Steering: {steering:.3f}, Throttle: {throttle:.3f}", end="\r")
            # time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
    finally:
        stop_threads = True
        
        # Wait for thread to terminate
        if lidar_thread.is_alive():
            lidar_thread.join(timeout=2.0)
            
        lidar_device.terminate()
        myCar.terminate()
        print("Program terminated")
        sys.exit(0)

if __name__ == '__main__':
    main()