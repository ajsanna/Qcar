'''This example demonstrates how to read and display data from the QCar Lidar
'''
import time
import matplotlib.pyplot as plt
import numpy as np
from pal.products.qcar import QCarLidar, IS_PHYSICAL_QCAR
from lidar2 import Lidar



# polar plot object for displaying LIDAR data later on



# def __init__(
#             self,
#             numMeasurements=384,
#             rangingDistanceMode=0,
#             interpolationMode=1,
#             interpolationMaxDistance=1,
#             interpolationMaxAngle=0,
#             enableFiltering=True,
#             angularResolution=1*np.pi/180
#         ):


runTime = 20.0 # seconds
# Initialize a Lidar device (e.g. RPLidar)
lidar_device = Lidar(type='RPLidar')
with QCarLidar(numMeasurements=80,rangingDistanceMode=2, interpolationMode=1, interpolationMaxDistance=0.02, interpolationMaxAngle=0.01) as myLidar:
  t0 = time.time()
  plt.ion()  # Turn on interactive mode
  fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
  start_time = time.time()
  
  
  #while True:
  while time.time() - t0  < runTime:
    plt.cla()
  
    # Clear previous plot
    ax.clear()
    lidar_device.read()
    # Access measurement data
    print((lidar_device.distances, lidar_device.angles))

    
    # Check if data is valid
    if hasattr(myLidar, 'angles') and hasattr(myLidar, 'distances'):
      
      print(f"Angles: {len(myLidar.angles)}, Distances: {len(myLidar.distances)}")
      ax.scatter(myLidar.angles, myLidar.distances, marker='.')
      ax.set_theta_zero_location("W")
      ax.set_theta_direction(-1)
        
      plt.pause(0.1)  # Allow the plot to update
    
  plt.ioff()  # Turn off interactive mode
  plt.show()  # Show the final plot after the loop ends
    
    
    
    
    
    
    
    
    
 
