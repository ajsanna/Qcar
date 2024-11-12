import argparse

import time
import subprocess
import threading
import sys
import socket
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'  # or any {'0', '1', '2'}
import numpy as np
import cv2
from pal.products.qcar import QCar, QCarRealSense
from pal.utilities.vision import Camera2D
from datetime import datetime



import payload

stopthread = False

myCar = QCar(readMode=0)
max_throttle = 0.075
min_throttle = -0.075
max_steering = 0.5
min_steering = -0.5
LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
rear_cam = False
right_cam = False
left_cam = False

steering = 0
throttle = 0
currentGear = 1
speeds = [0, .075, .125, .2, .225, .3, .35]
reverse = False

PORT = 38821  # Port to listen on (non-privileged ports are > 1023)
camera_right = Camera2D(cameraId="0",frameWidth=420,frameHeight=220,frameRate=30)
camera_back = Camera2D(cameraId="1",frameWidth=420,frameHeight=220,frameRate=30)
camera_left = Camera2D(cameraId="2",frameWidth=420,frameHeight=220,frameRate=30)
camera_front = Camera2D(cameraId="3",frameWidth=420,frameHeight=220,frameRate=30)



def get_wifi_networks():
   while(True):
    """Gets the list of available Wi-Fi networks."""

    command = "nmcli device wifi list"
    output = subprocess.check_output(command, shell=True)
    decoded_data = output.decode('utf-8')
    lines = decoded_data.splitlines()
    for line in lines:
            print(line.strip())
    print("\n" + "-" * 50)
    print("\n")
    time.sleep(2)

def drive():   
    print("Driving Starting...")
    left_indicator = 0 
    right_indicator = 0
   
    global throttle, steering, reverse, currentGear, speeds, rear_cam_view, rear_cam, left_cam, right_cam
    
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('', PORT))
        global stopthread
        
        while True:
            if stopthread:
                break

            data = s.recvfrom(100)[0].decode('utf-8')
            if not data:
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
                #print("throttle: " + str(throttle))
                #print(event)

                if event == 1536:
                    #IF Axis is Steering Wheel
                    #print(float(buffer[2]))
                    #print(steering)
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
                        
                        #th = 0.6 * ((abs(float(buffer[3]) -1 ) /2 ) * 0.2)
                        th = (float(buffer[3])) / 2 - 0.5
                        throttle = th * max_throttle
                        #if th < 0:
                            #throttle = max(th, min_throttle)
                        #else:
                          #throttle = min(th, max_throttle)
                    
                    #IF Axis is Brake
                    elif float(buffer[2]) == 2:
                        continue 
                        #Implement Brake algorithm

                if event == 1539:
                    #print(float(buffer[2]))
                    if float(buffer[2]) == 5:
                        reverse = False
                        currentGear = currentGear - 1 if currentGear > 1 else currentGear
                        print("Gear: " + str(currentGear))
                        max_throttle = speeds[currentGear]
                    elif float(buffer[2]) == 4:
                        currentGear = currentGear + 1 if currentGear < 6 else currentGear
                        print("Gear: " + str(currentGear))
                        max_throttle = speeds[currentGear]
                        reverse = False
                    elif float(buffer[2]) == 0:
                        print("A Pressed")
                        if rear_cam == False: 
                            rear_cam = True
                            print("Rear Cam: ON")
                            rear_cam_view.start()
                            #rear_cam_view.join()
                        # else: 
                        #     rear_cam = False
                        #     print("Rear Cam: OFF")
                    elif float(buffer[2]) == 2:
                        print("X Pressed")
                        if left_cam == False: 
                            left_cam = True
                            print("Left Cam: ON")
                            left_cam_view.start()
                    elif float(buffer[2]) == 1:
                        print("B Pressed")
                        if right_cam == False: 
                            right_cam = True
                            print("Right Cam: ON")
                            right_cam_view.start()
                            
                            
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
                    # elif float(buffer[2]) == 1:
                    #     print("Right Cam Flipped")
                       
                if reverse:
                    if throttle > 0:
                        throttle *= -1
                else:
                    throttle = abs(throttle)
                myCar.write(throttle=throttle, steering=steering, LEDs = LEDs)
                    

            except Exception as e:
                print("Invalid Packet Size")
                print(e)

    print("Terminated Driving")
    
def camPreview(camIDs = ["front"]):
    global rear_cam
    running = True
    while running:
        # if "back" in camIDs and rear_cam == False: 
        #     print("Breaking")
        #     camera_back.terminate()
        #     cv2.destroyWindow("Camera Back")
        #     running = False
        
        if "front" in camIDs:
            camera_front.read()
            if camera_front is not None:
                #cv2.namedWindow("Camera Front", cv2.WINDOW_NORMAL)
                #cv2.resizeWindow("Camera Front", 900, 900)
                cv2.imshow("Camera Front", camera_front.imageData)
        if "left" in camIDs:
            camera_left.read()
            if camera_left is not None:
                cv2.imshow("Camera Left", camera_left.imageData)
        if "right" in camIDs:
            camera_right.read()
            if camera_right is not None:
                cv2.imshow("Camera Right", camera_right.imageData)
        if "back" in camIDs:
            camera_back.read()
            if camera_back is not None:
                #print("image found")
                cv2.imshow("Camera Back", camera_back.imageData)
        key = cv2.waitKey(100)
        if key == 27:  # exit on ESC
            camera_front.terminate()
            cv2.destroyWindow("Camera Front")



keyControl = threading.Thread(target=drive)
cameraAccess = threading.Thread(target=camPreview,args=[["front"]])
left_cam_view = threading.Thread(target=camPreview,args=[["left"]])
right_cam_view = threading.Thread(target=camPreview,args=[["right"]])
rear_cam_view = threading.Thread(target=camPreview,args=[["back"]])

def main():
    import ctypes
    ctypes.CDLL('libX11.so.6').XInitThreads()
    global keyControl, cameraAccess
    #keyControl = threading.Thread(target=drive)
    #cameraAccess = threading.Thread(target=camPreview,args=[["front"]])
    #rear_camera_view = threading.Thread(target=camPreview,args=["back"])
    network = threading.Thread(target=get_wifi_networks)
    
    try:
        keyControl.start()
        cameraAccess.start()
        network.start()
    except KeyboardInterrupt:
            myCar.terminate()
            # exitPygame
            pygame.quit()
            sys.exit()


    

if __name__ == '__main__':
    main()