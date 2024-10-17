import numpy as np

# Load the .npy file
path = "/home/nvidia/Documents/Quanser/ClassHopper/DepthImages/"
loaded_array = np.load(path + 'sample_30.npy')

# Print the shape and data type of the loaded array
print(f"Loaded array shape: {loaded_array.shape}")
print(f"Data type: {loaded_array.dtype}")

# Optionally, you can display the image using OpenCV
import cv2

# If the array is an image (e.g., in RGB format)
cv2.imshow('Loaded Image', loaded_array)
cv2.waitKey(0)
cv2.destroyAllWindows()