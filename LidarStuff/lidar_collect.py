import pygame
from pal.products.qcar import QCar
from pal.utilities.math import *
import numpy as np
import time
import cv2
import threading
import os
from datetime import datetime
import matplotlib.pyplot as plt
from lidar2 import Lidar

# Open the catalog file for writing
#catalog = open("lidar_catalogs2.txt", "w")
#catalog.write("Count, Angles, Distances\n")

# Initialize parameters
runTime = 5.0  # Duration to run the LiDAR reading
lidar_device = Lidar(type='RPLidar')
global_count = [0]  # Use a list to hold the count

def lidar(global_count, angles, distances, catalog): 
    if distances is not None and angles is not None: 
        data = f"{global_count[0]}, {angles.tolist()}, {distances.tolist()}\n"
        #catalog.write(data)
        global_count[0] += 1  # Increment the count

# Set up the polar plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

print(lidar_device.angles, lidar_device.distances)
t0 = time.time()

try:
    while time.time() - t0 < runTime:
        plt.cla()  # Clear the current axes

        lidar_device.read()  # Read data from the LiDAR

        # Check if data is valid
        if lidar_device.distances is not None and lidar_device.angles is not None:
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
    print(lidar_device.angles, lidar_device.distances)
    lidar_device.terminate()  # Ensure the device is terminated correctly
    catalog.close()  # Close the catalog file
