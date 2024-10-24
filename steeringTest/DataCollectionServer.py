import argparse
from pal.utilities.vision import Camera2D
from pal.products.qcar import QCarRealSense
import threading
import sys
import socket
import os
from datetime import datetime

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'  # or any {'0', '1', '2'}

import cv2
from pal.products.qcar import QCar, QCarRealSense

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
image_skipper = 0
global_count = 0
# global_index = 0
max_throttle = 0.2
min_throttle = -0.2
max_steering = 0.5
min_steering = -0.5

steering = 0
throttle = 0
reverse = False

PORT = 38821  # Port to listen on (non-privileged ports are > 1023)
camera_front = Camera2D(cameraId="3",frameWidth=420,frameHeight=220,frameRate=30) 
folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")
# Create the full path for the new folder
path = "/media/378B-14FD/Collected_Images/"
full_folder_path = os.path.join(path, folder_name)
os.makedirs(full_folder_path)  

def drive():
    print("Driving Starting...")
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
                #print(event)

                if event == 1536:
                    #IF Axis is Steering Wheel
                    #print(float(buffer[2]))
                    if float(buffer[2]) == 0:
                        steer = -1* float(buffer[3]) * 2
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
                        
                        th = 0.6 * ((abs(float(buffer[3]) -1 ) /2 ) * 0.2)
                        if th < 0:
                            throttle = max(th, min_throttle)
                        else:
                            throttle = min(th, max_throttle)
                    
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
                        reverse = True
                    elif float(buffer[2]) == 12:
                        # First Gear Selected 
                        print("1st Gear Selected")
                        reverse = False

                if reverse:
                    if throttle > 0:
                        throttle *= -1
                else:
                    throttle = abs(throttle)
                myCar.write(throttle=throttle, steering=steering)
                if(throttle != 0 and image_skipper % 5 == 0):
                    camPreview(["front"], 1, steering, throttle)
                    
                    

            except Exception as e:
                print("Invalid Packet Size")
                print(e)

    print("Terminated Driving")
    
    
    
def camPreview(camIDs, global_count, steering, throttle):
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
        #catalog.write(data)
        global_count += 1
        
def snapshot(image, global_count):
    img = image
   
    
    # Create the new folder
    

    img_name = 'sample_' + str(global_count) + '.jpg'

    #write the image to the file at path specified above
    print("Saving Image")
    cv2.imwrite(os.path.join(full_folder_path, img_name), img)


def main():
    def exiting():
        # camera_realsense_rgb.terminate()
        # if videoRecording:
        #     out.release()
        # cv2.destroyWindow("RealSense Camera")
        global stopthread
        stopthread=True
        t2.join()
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

    try:
        t2.start()
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
        exiting()


    

if __name__ == '__main__':
    main()