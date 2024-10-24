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

catalog = open("lidar_catalogs1.txt", "w")
catalog.write("Count, Angles, Distances\n")
global_count = 0

# Add this function to convert polar to cartesian
def polar_to_cartesian(angles, distances):
    x = distances * np.cos(np.radians(angles))
    y = distances * np.sin(np.radians(angles))
    return x, y

def lidar(global_count, angles, distances, catalog):
    if int(global_count) >= 0:
        if distances is not None and angles is not None:
            # Flatten angles and distances if needed
            angles = np.array(angles).flatten()
            distances = np.array(distances).flatten()

            # Convert to Cartesian
            x, y = polar_to_cartesian(angles, distances)  
            
            # Log x, y coordinates
            data = f"{global_count}, {x.tolist()}, {y.tolist()}\n"
            catalog.write(data)
            global_count += 1
            
# Initialize parameters
runTime = 5.0  # Duration to run the LiDAR reading
lidar_device = Lidar(type='RPLidar')

# Set up the polar plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

t0 = time.time()

# Inside your LiDAR reading loop
try:
    while time.time() - t0 < runTime:
        plt.cla()

        lidar_device.read()  # Read data from the LiDAR

        if lidar_device.distances is not None and lidar_device.angles is not None:
            ax.scatter(lidar_device.angles, lidar_device.distances, marker='.')
            ax.set_theta_zero_location("W")
            ax.set_theta_direction(-1)
            ax.set_title("LiDAR Scan Data", va='bottom')

            # Pause for plotting and call your logging function
            plt.pause(0.1)
            lidar(global_count, lidar_device.angles, lidar_device.distances, catalog)

finally:
    plt.ioff()
    plt.show()
    lidar_device.terminate()
