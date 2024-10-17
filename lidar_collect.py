# Import Lib
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

import time 
import matplotlib.pyplot as plt 
from lidar2 import Lidar

catalog = open("lidar_catalogs.txt", "w")
catalog.write("Count, Angles, Distances\n")
global_count = 0

def lidar(global_count, angles, distances, catalog): 
  if int(global_count) >= 0:
    if distances is not None and angles is not None: 
      data = str(global_count) + ", " + str(angles) + ", " + str(distances) + "\n"
      catalog.write(data)
      global_count+=1 

# Initialize parameters
runTime = 5.0  # Duration to run the LiDAR reading
lidar_device = Lidar(type='RPLidar')

# Set up the polar plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

t0 = time.time()

try:
    while time.time() - t0 < runTime:
        plt.cla()  # Clear the current axes

        lidar_device.read()  # Read data from the LiDAR

        # Check if data is valid
        if lidar_device.distances is not None and lidar_device.angles is not None:
            # Convert distances from meters to a suitable range if needed
            # distances = lidar_device.distances.flatten()  # Flatten if necessary
            # angles = lidar_device.angles.flatten()
            
            ax.scatter(lidar_device.angles, lidar_device.distances, marker='.')
            print(lidar_device.angles, lidar_device.distances)
            ax.set_theta_zero_location("W")
            ax.set_theta_direction(-1)
            ax.set_title("LiDAR Scan Data", va='bottom')
            
            plt.pause(0.1)  # Pause to allow the plot to update
            lidar(global_count, lidar_device.angles, lidar_device.distances, catalog)
finally:
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show the final plot after the loop ends
    lidar_device.terminate()  # Ensure the device is terminated correctly

        

# distances = lidar_device.distances
# angles = lidar_device.angles



