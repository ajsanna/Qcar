
"""
This program interacts with a QCar robot equipped with a RealSense depth camera to capture and process depth images. It provides two main functions: continuously capturing depth data in a loop and saving the images to disk, and taking a single depth snapshot on demand. The depth data is normalized based on the maximum possible distance of the camera and saved as NumPy arrays for further processing or analysis.

### Key Features:

1. **Depth Camera Integration**:
   - The program uses the `QCarRealSense` class, which interfaces with the RealSense depth camera to capture depth images. These depth images represent the distance of objects in the camera's field of view.
   - The depth camera operates within a defined maximum distance (`max_distance`), which is set to 10 meters by default, ensuring that values beyond this range are normalized accordingly.

2. **Continuous Depth Image Capture** (`DepthCam` function):
   - The `DepthCam` function continuously captures depth data from the RealSense camera in a loop.
   - The program reads the depth image data and normalizes it by dividing by the `max_distance` to scale the pixel values between 0 and 1, representing the relative distances of objects.
   - Each depth image is saved as a `.npy` (NumPy array) file in the specified path for later use or analysis.
   - The captured depth image is displayed in a window labeled 'My Depth' using OpenCV (`cv2.imshow`), and the program waits for 100 milliseconds before capturing the next frame.
   - A global count (`global_count`) is used to incrementally track the images being captured, though it's not used for naming in the current implementation (fixed name 'sample_.npy').

3. **Single Depth Snapshot** (`takePhoto` function):
   - The `takePhoto` function allows for capturing a single depth snapshot on demand.
   - The function reads the depth image, normalizes the pixel values, and saves it as a `.npy` file.
   - The saved depth data is then displayed in a window using OpenCV, and the program waits for the user to press any key before closing the window and completing the process.

4. **Depth Image Storage and File Management**:
   - Depth images are saved in the specified folder (`path`), with the file name `sample_.npy`. These files store the normalized depth data as NumPy arrays, which makes it easy to load and process the data later.
   - The program uses `np.save` to save the depth images, which stores the data in an efficient binary format that can later be loaded with `np.load` for further analysis.

5. **Visualization**:
   - The program uses OpenCV to visualize the depth images in real-time, displaying them in a window labeled 'My Depth' during continuous capture and 'Loaded Image' when a single snapshot is taken.
   - The depth images are displayed with their normalized pixel values (scaled between 0 and 1) and will appear as a gradient representing distance.

6. **Customization**:
   - The maximum capture distance for depth measurements can be modified by adjusting the `max_distance` variable, allowing for fine-tuning depending on the application's specific needs.

### Workflow:
- The program initializes the RealSense camera (`QCarRealSense`) and enters either the continuous capture mode (in `DepthCam`) or the single photo mode (in `takePhoto`).
- In the continuous capture mode, depth images are read, normalized, and saved in an incremental manner. The program also shows the current depth image using OpenCV.
- In the single photo mode, one depth snapshot is captured, normalized, saved to the disk, and displayed.

### Data Output:
- Depth data is saved as `.npy` files in the specified directory (`path`), with each file containing a normalized depth map of the scene.
- The depth images are stored as NumPy arrays, making them suitable for later processing with machine learning or computer vision algorithms.

Authored by: 
    - Alex Sanna ajsanna@cpp.edu 
    - Matthew Baldivino mabaldivino@cpp.edu 
    - Sebastian Cursaro scursaro@cpp.edu
    - Joseph Bui jhbui@cpp.edu
    - Reyna Nava nvnava@cpp.edu
    - Rich Chea richweichea@cpp.edu

Latest Revision: February 2025
"""

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