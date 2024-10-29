import argparse

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
# import DetectLane as DetectLane

# parser = argparse.ArgumentParser(
#                                     prog='Q Car Contorl Handler',
#                                     description='Handles Image Processing and QCar Control'
#                                 )
# parser.add_argument(
#                     "-v", "--video",
#                     action='store_true',
#                     help="Enable or Disable Video Capture."
#                     )

# args = parser.parse_args()
# videoRecording = False
# WIDTH = 480
# HEIGHT = 240
# if args.video:
#     videoRecording = True

# if videoRecording:
#     # Define the codec and create a VideoWriter object
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec as needed (e.g., 'XVID')
#     out = cv2.VideoWriter('video.mp4', fourcc, fps=30.0, frameSize=(WIDTH, HEIGHT))  # 'output.mp4' is the output file name

#     # Add this line before the main loop to start recording
#     out.open('video.mp4', fourcc, fps=30.0, frameSize=(WIDTH, HEIGHT))

stopthread = False

myCar = QCar(readMode=0)
max_throttle = 0.075
min_throttle = -0.075
max_steering = 0.5
min_steering = -0.5
LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])

steering = 0
throttle = 0
reverse = False

PORT = 38821  # Port to listen on (non-privileged ports are > 1023)
camera_front = Camera2D(cameraId="3",frameWidth=420,frameHeight=220,frameRate=30)

def drive():   
    print("Driving Starting...")
    left_indicator = 0 
    right_indicator = 0
    global throttle, steering, reverse
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
                print("throttle: " + str(throttle))
                #print(event)

                if event == 1536:
                    #IF Axis is Steering Wheel
                    #print(float(buffer[2]))
                    print(steering)
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

    while True:
        if "front" in camIDs:
            camera_front.read()
            if camera_front is not None:
                cv2.imshow("Camera Front", camera_front.imageData)
        key = cv2.waitKey(100)
        if key == 27:  # exit on ESC
            camera_front.terminate()
            cv2.destroyWindow("Camera Front")
    
def main():
    keyControl = threading.Thread(target=drive)
    cameraAccess = threading.Thread(target=camPreview,args=["front"])
    
    try:
        keyControl.start()
        cameraAccess.start()
    except KeyboardInterrupt:
            myCar.terminate()
            # exitPygame
            pygame.quit()
            sys.exit()


    '''def exiting():
        # camera_realsense_rgb.terminate()
        # if videoRecording:
        #     out.release()
        # cv2.destroyWindow("RealSense Camera")
        global stopthread
        stopthread=True
        t2.join()
        t1.join()
        # Close all windows
        cv2.destroyAllWindows()
        quit()
    #Setup Camera and Predition Model for Lane Detection
    # camera_realsense_rgb = QCarRealSense(mode='RGB')
    # model = DetectLane.DetectLane()

    
    # Start Thread For Steering
    # Accepts Steering Inputs Over UDP
    # Can Be Easily Modified to Accept Inputs from any Source
    t2 = threading.Thread(target=drive)
    t1 = threading.Thread(target= camPreview)

    try:
        t2.start()
        t1.start()
        while t2.is_alive():

            # camera_realsense_rgb.read_RGB()

            #Send Image for Predition
            # if camera_realsense_rgb is not None:
            #     frame = cv2.resize(camera_realsense_rgb.imageBufferRGB, (int(WIDTH), int(HEIGHT)))
            #     result = model.detectLanes(frame)
            #     if videoRecording:
            #         out.write(frame)
            #     cv2.imshow("RealSense Camera", result)


            key = cv2.waitKey(1)

            #If ESC is pressed begin termination sequence
            if key == 27:
                exiting()
                

    except Exception as e:
        print("Encountered Error:")
        print(e)
    finally:
        print("Exiting Main")
        exiting()'''


    

if __name__ == '__main__':
    main()