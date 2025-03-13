
'''
    This program was made to view camera feeds and perform preprocessing steps in live time. 
    You can adjust this code to do preprocessing in live time. 
    Code written: November 2023
    Revised: January 2025
    Authors: Alex Sanna, Matthew Baldivino - ajsanna@cpp.edu
'''

#imports 
import sys
import cv2
import numpy as np
import threading
import os
from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR
from pal.utilities.vision import Camera2D
from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR

#region of interest method designed by unknown - 2023
def region_of_interest(image):
    height, width = image.shape[:2]
    polygons = np.array([
    [(0,int(height/2+110)),(int(width/2),int(height/2)),(width,int(height/2+110))]]) #(200,height),(1100, height),(550, 250)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask,polygons,255)
    masked_image = cv2.bitwise_and(image,mask)
    return masked_image


# Lane detection using Hough Line Transform
def detect_lanes(frame):
    # Preprocess the frame (convert to grayscale and apply Gaussian blur)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection (Canny)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Define ROI (Region of Interest)
    cropped_roi = region_of_interest(edges)
    

    # Use Hough Line Transform to detect lines\
    lines_list =[]
    lines = cv2.HoughLinesP(
                cropped_roi, # Input edge image
                1, # Distance resolution in pixels
                np.pi/180, # Angle resolution in radians
                threshold=70, #50  Min number of votes for valid line
                minLineLength=1, #1  Min allowed length of line
                maxLineGap=10 #15 Max allowed gap between line for joining them
                )
    
    
    # Draw the detected lines on the frame
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(gray, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.imshow("GrayScale",gray)
            cv2.imshow("Cropped ROI",cropped_roi)

#initialize camera objects with Camera2D
camera_right = Camera2D(cameraId="0",frameWidth=420,frameHeight=220,frameRate=30)
camera_back = Camera2D(cameraId="1",frameWidth=420,frameHeight=220,frameRate=30)
camera_left = Camera2D(cameraId="2",frameWidth=420,frameHeight=220,frameRate=30)
camera_front = Camera2D(cameraId="3",frameWidth=420,frameHeight=220,frameRate=30)
depth_cam = QCarRealSense()      

'''
    region of interest method designed for blacking out irrelevant sections of images captured. 
    Author Reyna Nava - rvnava@cpp.edu
    Revised: December 2024
'''
def identify_lane(frame):

    # remove the QCAR bumper from view
    height, width = frame.shape[:2]
    cropped_height = int(height*0.85)
    frame = frame[:cropped_height,:]

    # store image size
    height, width = frame.shape[:2]

    # crop the field of view
    #poly_shape = np.array([(0,height),(int(width*0.5),0),(int(width*0.5),0),(width,height),],dtype=np.int32)
    #cv2.fillPoly(mask,[poly_shape],(255))
    #cv2.fillPoly(mask,ellipse,(255))

    #return frame 
    # take img hsv color space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # lane color range (orange)
    opaque_orange = np.array([255,204,204])
    saturated_orange = np.array([153,76,0])
    
    # saturate the lane color
    orange_mask = cv2.inRange(hsv_image,opaque_orange,saturated_orange)
    orange_region = cv2.bitwise_and(frame,frame,mask=orange_mask)
    hsv_image[:,:,1] = np.clip(hsv_image[:,:,1]*1.25,0,255)
    hsv_image[:,:,2] = np.clip(hsv_image[:,:,2]*1.25,0,255)

    # convert back 
    saturated_frame = cv2.cvtColor(hsv_image,cv2.COLOR_HSV2BGR)
    
    # blurr the background + apply edge detection
    blurred_image = cv2.GaussianBlur(saturated_frame,(3,3),1.5)
    edges = cv2.Canny(blurred_image,50,150)
    mask = np.zeros((height,width),dtype=np.uint8)
    ellipse = cv2.ellipse(mask,(width//2,height -50),(int(width//2),int(height//4)),0,0,360,255, -20)
    frame = cv2.bitwise_and(edges,edges,mask=mask)

    #gray_scaled = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    return frame




'''
    main method for viewing camera feeds. used by the main method to display feeds to screen. 
'''
def camPreview(camIDs):

    while True:
        if "front" in camIDs:
            camera_front.read()
            if camera_front is not None:
                img = identify_lane(camera_front.imageData)
                cv2.imshow("Camera Front", img)
        if "back" in camIDs:
            camera_back.read()
            if camera_back is not None:
                detect_lanes(camera_back.imageData)
                cv2.imshow("Camera Back", camera_back.imageData)
        if "left" in camIDs:
            camera_left.read()
            if camera_left is not None:
                detect_lanes(camera_left.imageData)
                cv2.imshow("Camera Left", camera_left.imageData) 
        if "right" in camIDs:
            camera_right.read()
            if camera_right is not None:
                detect_lanes(camera_right.imageData)
                cv2.imshow("Camera Right", camera_right.imageData)
        if 'depth' in camIDs:
            if depth_cam is not None: 
                max_distance = 10
                depth_cam.read_depth()
                image = depth_cam.imageBufferDepthPX/max_distance
                arr_image = np.asanyarray(image)
                cv2.imshow('Depth Image', arr_image)
                
        key = cv2.waitKey(100)
        if key == 27:  # exit on ESC
            camera_front.terminate()
            cv2.destroyWindow("Camera Front")
            camera_right.terminate()
            cv2.destroyWindow("Camera Right")
            camera_back.terminate()
            cv2.destroyWindow("Camera Back")
            camera_left.S()
            cv2.destroyWindow("Camera Left")
            #depth_cam.terminate()
            #cv2.destroyWindow("Depth Image")
            break
            

if __name__ == "__main__":
    
    try:
        print("To exit, press ESC")
        while True:
            camPreview(["front","left","right","back","depth"])
            

    except KeyboardInterrupt:
            sys.exit()