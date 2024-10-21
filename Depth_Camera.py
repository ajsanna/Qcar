
'''hardware_test_intelrealsense.py

This example demonstrates how to read and display depth & RGB image data
from the Intel Realsense camera.
'''
import os
import numpy as np
import cv2
import pyrealsense2 as rs
from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR
path = "/home/nvidia/Documents/Quanser/ClassHopper/DepthImages/"
myCam = QCarRealSense()

def DepthCam():
  import cv2
  from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR
  max_distance = 10 # meters (for depth camera)
  global_count = 0
  with QCarRealSense() as myCam:
    while True: 
      myCam.read_depth()
      img_name = 'sample_' + '.npy'
      image = myCam.imageBufferDepthPX/max_distance
      arr_image = np.asanyarray(image)
      arr_image = arr_image.astype(np.float32)
      np.save(os.path.join(path, img_name), arr_image)
      cv2.imshow('My Depth', image)
      cv2.waitKey(100)
      global_count+=1
      
def takePhoto():
  max_distance = 10
  myCam.read_depth()
  img_name = 'sample_' + '.npy'
  image = myCam.imageBufferDepthPX/max_distance
  arr_image = np.asanyarray(image)
  arr_image = arr_image.astype(np.float32)
  np.save(os.path.join(path, img_name), arr_image)
  cv2.imshow('Loaded Image', arr_image)
  cv2.waitKey(0)
                        
takePhoto()
#DepthCam()