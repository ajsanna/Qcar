import argparse
from pal.utilities.vision import Camera2D
from pal.products.qcar import QCarRealSense
import threading
import sys
import socket
import os
from datetime import datetime
import numpy as np

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'  # or any {'0', '1', '2'}

import cv2
from pal.products.qcar import QCar, QCarRealSense

import payload

stopthread = False

myCar = QCar(readMode=0)

max_throttle = 0.2
min_throttle = -0.2
max_steering = 0.5
min_steering = -0.5
LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])

steering = 0
throttle = 0
image_skipper = 0
reverse = False

PORT = 38822  # Port to listen on (non-privileged ports are > 1023)
camera_front = Camera2D(cameraId="3",frameWidth=420,frameHeight=220,frameRate=30) 
folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")

# Create the full path for the new folder
path = "/media/SANDISK1/Images"
full_folder_path = os.path.join(path, folder_name)
os.makedirs(full_folder_path)  

def drive():
    print("Driving Starting...")
    
     #file management for memory purposes
    catalog = open(r"images_catalogs.txt", "a")
    tracker = open(r"index_tracker.txt", "r")
    global_index = tracker.read()
    global_index = int(global_index)
    global_count = global_index
    print("starting index: " + str(global_index))
    tracker.close()
    
    global throttle, steering, reverse, image_skipper
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('', PORT))
        s.setblocking(False)
        global stopthread

        while True:
            #print("ITERATION")
            if stopthread:
                break

            try:
                data = s.recvfrom(100)[0].decode('utf-8')
                #print(data)
                #print(data)
                if not data:
                    #print("NOT DATA")
                    pass

                packet = payload.payload_handler(data)
                buffer = []
                try:
                    '''
                    /*##################################################################################*/
                    /*|    Controller ID    |    Event    |    Event Dimension     |    Event Value    |*/
                    /*|        4 Bit        |    4 Bit    |         8 Bit          |       4 Bit       |*/
                    /*##################################################################################*/
                    '''
                    
                    '''
                    Read Payload Data
                    '''
                    if packet.read(buffer, 4) == -1 or \
                    packet.read(buffer, 4) == -1 or \
                    packet.read(buffer, 8) == -1 or \
                    packet.read(buffer, 8) == -1:
                        print("Warning: Packet Length Too short")
                        continue
                    
                    event = int(buffer[1])
                    #print(event)

                    if event == 1536:
                        #IF Axis is Steering Wheel
                        #print(float(buffer[2]))
                        if float(buffer[2]) == 0:
                            steer = (-1* float(buffer[3])) / 1.5
                            if abs(steering - steer) < 0.05:
                                continue

                            if (steering == min_steering and steer < min_steering
                                or
                                steering == max_steering and steer > max_steering):
                                continue

                            if steer < 0:
                                steering = max(steer, min_steering)
                            else:
                                steering = min(steer, max_steering)
                            
                        #IF Axis is Throttle
                        elif float(buffer[2]) == 1:
                            
                            th = (float(buffer[3])) / 2 - 0.5
                            throttle = th * max_throttle
                        
                        #IF Axis is Brake
                        elif float(buffer[2]) == 2:
                            continue 
                            #Implement Brake algorithm

                    if event == 1539:
                        #Sprint(float(buffer[2]))
                        if float(buffer[2]) == 5:
                            print("Reverse Triggered")
                            reverse = True
                        elif float(buffer[2]) == 4:
                            print("Forward Triggered")
                            reverse = False
                        elif float(buffer[2]) == 0:
                            steering = 0
                            throttle = 0
                            reverse = False
                        elif float(buffer[2]) == 11:
                            # Reverse Gear selected. 
                            print("Reverse Gear Selected")
                            min_throttle = -.075
                            max_throttle = .075
                            reverse = True
                        elif float(buffer[2]) == 12:
                            # First Gear Selected 
                            print("1st Gear Selected")
                            max_throttle = .075
                            reverse = False
                        elif float(buffer[2]) == 13:
                            # First Gear Selected 
                            print("2nd Gear Selected")
                            max_throttle = .125
                            reverse = False
                        elif float(buffer[2]) == 14:
                            # First Gear Selected 
                            print("3rd Gear Selected")
                            max_throttle = .2
                            reverse = False
                        elif float(buffer[2]) == 15:
                            # First Gear Selected 
                            print("4th Gear Selected")
                            max_throttle = .225
                            reverse = False
                        elif float(buffer[2]) == 16:
                            # First Gear Selected 
                            print("5th Gear Selected")
                            max_throttle = .3
                            reverse = False
                        elif float(buffer[2]) == 17:
                            # First Gear Selected 
                            print("6th Gear Selected")
                            max_throttle = .35
                            reverse = False
                        elif float(buffer[2]) == 6:
                            LEDs[6] = 1 if LEDs[6] == 0 else 0
                            LEDs[7] = 1 if LEDs[7] == 0 else 0
                            LEDs[4] = 1 if LEDs[4] == 0 else 0
                            
                        elif float(buffer[2]) == 10:
                            print("Saving and shutting down.")
                            catalog.close()
                            tracker.close()
                            print("index upon shutdown: " + str(global_count))
                            tracker_update = open(r"index_tracker.txt", "w")
                            tracker_update.write(str(global_count))
                            tracker_update.close()
                            #print("Shutting Down")
                            exit(0)

                    if reverse:
                        if throttle > 0:
                            throttle *= -1
                    else:
                        throttle = abs(throttle)
                        
                    myCar.write(throttle=throttle, steering=steering, LEDs = LEDs)
                    if throttle != 0 and image_skipper % 5000 ==0:
                        camPreview(["front"], global_count, steering, throttle, catalog)
                        global_count = global_count+1
                        
                    image_skipper += 1
                    
                        

                except Exception as e:
                    print("Invalid Packet Size")
                    print(e)
            except Exception as E:
                if throttle != 0  and image_skipper % 5000 ==0:
                    camPreview(["front"], global_count, steering, throttle, catalog)
                    global_count = global_count+1
                image_skipper +=1

    print("Terminated Driving")
    
    
    
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
        
def snapshot(image, global_count):
    img = image
   
    
    # Create the new folder
    

    img_name = 'sample_' + str(global_count) + '.jpg'

    #write the image to the file at path specified above
    print("Saving Image")
    cv2.imwrite(os.path.join(full_folder_path, img_name), img)


def main():
  drive()

if __name__ == '__main__':
    main()