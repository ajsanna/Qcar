'''from lidar2 import Lidar
import time
import matplotlib.pyplot as plt
import numpy as np

lidar_device = Lidar(type='RPLidar')

try:
    while True:
        lidar_device.read()
        print((lidar_device.distances, lidar_device.angles))
        time.sleep(1)  # Pause for a second between readings
        
        
except Exception as e:
    print(f"Error: {e}")
finally:
    lidar_device.terminate()
'''

import time
import numpy as np
import matplotlib.pyplot as plt
from lidar2 import Lidar

# Initialize parameters
runTime = 20.0  # Duration to run the LiDAR reading
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

finally:
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show the final plot after the loop ends
    lidar_device.terminate()  # Ensure the device is terminated correctly
