
"""
This program is designed to control and monitor a QCar robot equipped with various sensors and cameras, using a manual control interface built with Pygame. The main objective of the program is to provide real-time control of the QCar’s movement, including throttle and steering adjustments, as well as monitor and store sensor data from multiple cameras, including a depth sensor.

### Key Features:

1. **Manual Control Interface**:
   - The program allows manual control of the QCar's throttle and steering using the keyboard (WASD keys). The user can accelerate, decelerate, reverse, and steer the car in both directions.
   - The "R" and "S" keys control reverse and stop functionalities. The program dynamically adjusts the throttle when keys are pressed or released, ensuring smooth deceleration and acceleration.
   - The car's steering is controlled using the "A" (left) and "D" (right) keys, with steering limits applied to prevent over-rotation.
   - The LED indicators on the car are controlled based on the car's movement (turn signals for left/right turns, reverse indicators, and headlights). 
   - The car’s movement state (forward, reverse, stop) is visually represented through LED lights, providing feedback to the user.

2. **Camera Data Collection**:
   - The program utilizes multiple cameras (front, back, left, and right) to capture images from different angles of the environment surrounding the car.
   - Additionally, a depth camera (RealSense) is integrated, capturing depth information to generate a 3D representation of the environment around the car.
   - The program allows for selective image capture, depending on whether the user requests front, back, left, right, or depth images.
   - The captured images are saved in both JPG (for color images) and NPY (for depth data), and they are organized in folders with timestamps to ensure proper file management.
   - The camera preview function is designed to capture images periodically (e.g., every 10th loop iteration) and log them along with the current throttle and steering values.

3. **Data Logging and File Management**:
   - A catalog file (`images_catalogs.txt`) is used to log the captured images along with the car's status, including throttle, steering, and which cameras were used for image capture.
   - The program also manages an index tracker (`index_tracker.txt`), ensuring that images are saved with a unique index and that the index persists between runs.
   - Each image capture session is stored in a new folder, named with the current timestamp, located on an external drive or local storage.
   - Depth images are saved as NPY files, making them suitable for further processing and analysis.

4. **Real-Time LED Control**:
   - The program handles real-time LED control for the QCar. LEDs are used to indicate different car statuses:
     - Turn signals (left/right) based on steering direction.
     - Headlights that can be toggled on/off using the "H" key.
     - Reverse indicators when the car is in reverse mode.
     - A status LED for throttle control (indicating if the car is in reverse or not).
   - The turn signal LED behavior is controlled based on the steering direction, and the turn signal flashes at regular intervals when the car is turning.

5. **Throttling and Steering Adjustments**:
   - Throttle is adjusted incrementally using the "W" and "S" keys (for forward and backward movement), and it has limits to ensure that the car does not exceed safe speed ranges.
   - The steering is adjusted incrementally with the "A" and "D" keys. Steering limits are applied to prevent extreme turns, ensuring that the car’s behavior remains predictable and safe.

6. **Time-Based Control**:
   - The program allows for time-based execution by setting a `runTime` parameter, which controls the duration for which the car will run. The system operates in a loop and responds to real-time user inputs.
   - The program also includes a turn signal timer to manage the flashing intervals of the turn signal LEDs.

### Workflow:
- The program initializes Pygame and sets up the control interface, where the user can manually operate the car via the keyboard.
- It continuously monitors the keyboard inputs and adjusts throttle, steering, and LEDs accordingly.
- Based on the car's state and the selected cameras, it captures images and logs data (throttle, steering, and image information) at regular intervals.
- The camera feed from the front camera is displayed periodically, and depth data from the depth camera is captured and saved as NPY files.

### Files and Data:
- Images and depth data are saved into folders named with the timestamp when the program is run.
- The image catalog (`images_catalogs.txt`) keeps track of image names, camera IDs, and the car's throttle and steering data.
- The index tracker file (`index_tracker.txt`) keeps track of the last used index to ensure consistent and sequential image numbering across multiple runs.

This program serves as a comprehensive manual control interface for a QCar robot, integrating real-time movement control with camera-based data collection and logging for further analysis or machine learning applications.

Authored by: 
    - Alex Sanna ajsanna@cpp.edu 
    - Matthew Baldivino mabaldivino@cpp.edu 
    - Sebastian Cursaro scursaro@cpp.edu
    - Joseph Bui jhbui@cpp.edu
    - Reyna Nava nvnava@cpp.edu
    - Rich Chea richweichea@cpp.edu

Latest Revision: February 2025
"""



# import lib
import pygame
from pal.products.qcar import QCar
from pal.utilities.math import *
import numpy as np
import time
from pal.utilities.vision import Camera2D
import sys
import cv2
import numpy as np
import threading
import os
from datetime import datetime
from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR
from pal.utilities.vision import Camera2D
from pal.products.qcar import QCarRealSense
import sys

# create qcar
myCar = QCar(readMode=0)

def Drive():
    image_skipper = 1
    # init Pygame
    pygame.init()
    depth_image = False
    
    if len(sys.argv) > 1:
      if sys.argv[1] == "depth":
        depth_image = True
        print("Collecting Depth")
    else:
      print("No Depth Images")
      
    #file management for memory purposes
    catalog = open(r"images_catalogs.txt", "a")
    tracker = open(r"index_tracker.txt", "r")
    global_index = tracker.read()
    global_index = int(global_index)
    print("index: " + str(global_index))
    tracker.close()
    
    # setup screen size
    screen_width = 800
    screen_height = 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("QCar Manual Control")

    # set up +- increment and maximum
    deceleration_increment = .00001
    throttle_increment = 0.005
    steering_increment = 0.005
    max_throttle = 0.075
    min_throttle = -0.1
    max_steering = 0.5
    min_steering = -0.5
    MINIMUM_THROTTLE_EPSILON = .0000001

    # default speed and direction
    throttle = 0.0
    steering = 0.0

    def stop():
        throttle = 0.0
        steering = 0.0

    # main program
    runTime = 10.0 # seconds
    t0 = time.time()
    
    # Default LED status
    LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    isReverse = False
    headlights_on = True
    s_key_pressed = False

    turn_signal_timer = 0.0
    turn_signal_interval = 0.5
    turn_signal_state = 0 
    running = True
    while running == True:
    # while time.time() - t0  < runTime :
        # t = time.time()

        #listen for pygame Events in the event queue
        #print("while iteration")
        for event in pygame.event.get():
            #print("Event Detected")
            #if the window is closed, shut the car off
            if event.type == pygame.QUIT:
                catalog.close()
                tracker.close()
                running = False
                print("index upon shutdown: " + str(global_index))
                tracker_update = open(r"index_tracker.txt", "w")
                tracker_update.write(str(global_index))
                tracker_update.close()
                print("Shutting Down")
                exit(0)
            #if finger has been taken off the keys
            elif event.type == pygame.KEYUP:
                #maintain steering direction
                if event.key == pygame.K_a or  event.key == pygame.K_d:
                  steering = 0.0
                #if car is in drive and the throttle key is released
                if event.key == pygame.K_w and isReverse is False:
                  #reduce throttle amount
                  while throttle > MINIMUM_THROTTLE_EPSILON:
                    throttle -= .0000001
                  #car should come to a complete stop if it has reached minimum throttle before the throttle key is pressed again
                  throttle = 0
                #if car is in reverse and throttle key is released
                if event.key == pygame.K_w and isReverse is True:
                  #reduce throttle amount
                  while throttle < MINIMUM_THROTTLE_EPSILON:
                    throttle += .0000001
                  #car comes to complete stop upon reaching minimum throttle before the throttle key has been pressed
                  throttle = 0

        keys = pygame.key.get_pressed()

        # steering
        if keys[pygame.K_w]:
            if not isReverse:
                throttle += throttle_increment
                #prevents car from going over max throttle amount
                throttle = min(throttle, max_throttle)
                LEDs[4] = 0
                
            if isReverse:
                #prevents car from going under min throttle amount unless it is manually set to 0 like in deceleration algorithm
                throttle -= throttle_increment
                throttle = max(throttle, min_throttle)

        elif keys[pygame.K_s]:
            if throttle > 0 and not isReverse:
                throttle -= throttle_increment
                #prevent car from going under min throttle amount
                throttle = max(throttle, min_throttle)
                LEDs[4] = 1
                s_key_pressed = True  # Set flag when 'S' key is pressed
            elif throttle < 0 and isReverse:
                throttle += throttle_increment
                #prevent car from going under min throttle amount
                throttle = max(throttle, min_throttle)
            else:
                s_key_pressed = False  # Reset flag when 'S' key is released

        if not keys[pygame.K_s] and not s_key_pressed:
            LEDs[4] = 0

        if keys[pygame.K_r]:
            isReverse = True
            throttle = 0
            LEDs[4] = 0  # Turn off LED[4] when 'R' key is pressed
            LEDs[5] = 1
        elif keys[pygame.K_f]:
            isReverse = False
            throttle = 0
            LEDs[5] = 0

        if keys[pygame.K_d]:
            steering -= steering_increment
            #turning right is steering in the negative direction
            steering = max(steering, min_steering)
        elif keys[pygame.K_a]:
            steering += steering_increment
            #turning left is steering in the positive direction
            steering = min(steering, max_steering)
        
        if keys[pygame.K_q]:
            steering , throttle = 0.0 , 0.0
            LEDs[5] = 0
        
        if keys[pygame.K_h]:
            headlights_on= not headlights_on
            LEDs[6] = 1 if headlights_on else 0
            LEDs[7] = 1 if headlights_on else 0
        
        # update turnled timer
        turn_signal_timer += 0.01

        # based on direction and speed update led control
        if steering > 0.15:
            if turn_signal_timer >= turn_signal_interval:
                turn_signal_state = 1 - turn_signal_state
                turn_signal_timer = 0.0
            if turn_signal_state:
                LEDs[0] = 1
                LEDs[2] = 1
            else:
                LEDs[0] = 0
                LEDs[2] = 0
        elif steering < -0.15:
            if turn_signal_timer >= turn_signal_interval:
                turn_signal_state = 1 - turn_signal_state
                turn_signal_timer = 0.0
            if turn_signal_state:
                LEDs[1] = 1
                LEDs[3] = 1
            else:
                LEDs[1] = 0
                LEDs[3] = 0
        else:
            LEDs[0] = 0
            LEDs[1] = 0
            LEDs[2] = 0
            LEDs[3] = 0

        if throttle < 0:
            LEDs[5] = 1
        else:
            LEDs[5] = 0
        
        
        myCar.read_write_std(throttle=throttle, steering=steering, LEDs = LEDs)
        if(throttle != 0 and image_skipper % 10 == 0):
          camPreview(["front"], global_index, steering, throttle, catalog)
          global_index += 1
          
        '''if(throttle != 0 and image_skipper % 20 == 0 and depth_image):
          camPreview(["front", "depth"], global_index, steering, throttle, catalog)
          global_index += 1'''
  
        '''
        # update qcar control
        myCar.read_write_std(throttle=throttle, steering=steering, LEDs = LEDs)
        if(throttle != 0 and image_skipper % 5 == 0 ):
            #pass cam id, global image count, steering value, and throttle value to camPreview
            #camPreview will be used to take a photo of the surroundings as well as write the steering and throttle values to Catalog/images_catalogs.txt
            #global_index is a temporary field. We plan to read the global_index from index_tracker.txt for memory
            if len(sys.argv) == 2:
              if sys.argv[1] == "front":
                camPreview(["front"], global_index, steering, throttle, catalog)
                global_index += 1
              else:
                camPreview(["depth"], global_index, steering, throttle, catalog)
                global_index += 1
            else:
              camPreview(["front", "depth"], global_index, steering, throttle, catalog)
              global_index += 1
              
              
        '''
        image_skipper += 1
         # uncomment following line to see live time throttle steering data in terminal    
        #print(throttle, steering, isReverse, LEDs)

camera_right = Camera2D(cameraId="0",frameWidth=420,frameHeight=220,frameRate=30)
camera_back = Camera2D(cameraId="1",frameWidth=420,frameHeight=220,frameRate=30)
camera_left = Camera2D(cameraId="2",frameWidth=420,frameHeight=220,frameRate=30)
camera_front = Camera2D(cameraId="3",frameWidth=420,frameHeight=220,frameRate=30) 
depth_cam = QCarRealSense()      

def camPreview(camIDs, global_count, steering, throttle, catalog):
    if int(global_count) >= 0:
        data = str(global_count) + ", "
        if "front" in camIDs:
            camera_front.read()
            if camera_front is not None:
                #detect_lanes(camera_front.imageData)
                #cv2.imshow("Camera Front", camera_front.imageData)

                #snapshot function will take a picture
                snapshot(camera_front.imageData, global_count)
                img_name = 'sample_' + str(global_count) + '.jpg'
                #data = str(global_count) + ", " + img_name + ", " + str(steering) + ", " + str(throttle) + "\n"
                data += (img_name + ", ")
                #print(data)
                #catalog.write(data)
                #global_count+=1
        if "back" in camIDs:
            camera_back.read()
            if camera_back is not None:
                print("back")
                #detect_lanes(camera_back.imageData)
                #cv2.imshow("Camera Back", camera_back.imageData)
        if "left" in camIDs:
            camera_left.read()
            if camera_left is not None:
                print("left")
                #detect_lanes(camera_left.imageData)
                #cv2.imshow("Camera Left", camera_left.imageData) 
        if "right" in camIDs:
            camera_right.read()
            if camera_right is not None:
                print("right")
                #detect_lanes(camera_right.imageData)
                #cv2.imshow("Camera Right", camera_right.imageData)   
        if 'depth' in camIDs:
            if depth_cam is not None: 
                max_distance = 10
                depth_cam.read_depth()
                #take depth image and store it as an array
                image = depth_cam.imageBufferDepthPX/max_distance
                arr_image = np.asanyarray(image)
                img_name = 'sample_' + str(global_count) + '.npy'
                data += (img_name + ", ")
                #save depth image to the specified path below
                path = "/media/378B-14FD/Depth_Images/"
                np.save(os.path.join(path, img_name), arr_image)
       
          
        
        #write to catalog
        if 'depth' not in camIDs:
          data += "None, "        
        data += (str(steering) + ", " + str(throttle) + "\n")
        print(data)
        catalog.write(data)
        global_count += 1
        
folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")
# Create the full path for the new folder
path = "/media/SANDISK1/Images/"
full_folder_path = os.path.join(path, folder_name)
os.makedirs(full_folder_path)  

def snapshot(image, global_count):
    img = image
   
    
    # Create the new folder
    

    img_name = 'sample_' + str(global_count) + '.jpg'

    #write the image to the file at path specified above
    cv2.imwrite(os.path.join(full_folder_path, img_name), img)

Drive()