
'''hardware_test_intelrealsense.py

This example demonstrates how to read and display depth & RGB image data
from the Intel Realsense camera.
'''


def DepthCam():
  import cv2
  from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR
  max_distance = 10 # meters (for depth camera)
  with QCarRealSense() as myCam:
    while True: 
      myCam.read_depth()
      cv2.imshow('My Depth', myCam.imageBufferDepthPX/max_distance)
      cv2.waitKey(100)
                        
