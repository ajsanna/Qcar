'''This example demonstrates how to read and display data from the QCar Lidar
'''
import time
import matplotlib.pyplot as plt
from pal.products.qcar import QCarLidar, IS_PHYSICAL_QCAR


# polar plot object for displaying LIDAR data later on
ax = plt.subplot(111, projection='polar')
plt.show(block=False)

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
with QCarLidar(numMeasurements=80,rangingDistanceMode=2, interpolationMode=1, interpolationMaxDistance=0.02, interpolationMaxAngle=0.01) as myLidar:
t0 = time.time()
  while True:
    # while time.time() - t0  < runTime:
    plt.cla()
  
    # Capture LIDAR data
    myLidar.read()
  
    ax.scatter(myLidar.angles, myLidar.distances, marker='.')
    ax.set_theta_zero_location("W")
    ax.set_theta_direction(-1)
  
    plt.pause(0.1)
