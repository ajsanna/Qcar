

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
    max_throttle = 0.1
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
        for event in pygame.event.get():
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
                path = "/home/nvidia/Documents/Quanser/ClassHopper/DepthImages/"
                np.save(os.path.join(path, img_name), arr_image)
        data += (str(steering) + ", " + str(throttle) + "\n")
        print(data)
        catalog.write(data)
        global_count += 1
        
folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")
# Create the full path for the new folder
path = "/home/nvidia/Documents/Quanser/ClassHopper/Images/"
full_folder_path = os.path.join(path, folder_name)
os.makedirs(full_folder_path)  

def snapshot(image, global_count):
    img = image
   
    
    # Create the new folder
    

    img_name = 'sample_' + str(global_count) + '.jpg'

    #write the image to the file at path specified above
    cv2.imwrite(os.path.join(full_folder_path, img_name), img)
        
Drive()