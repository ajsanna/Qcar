
#sudo update-alternatives --config python3

import signal
import sys
import os
import numpy as np
import time
import cv2
from pal.products.qcar import QCar
from lidarThread import Lidar
from pal.utilities.math import *
from pal.utilities.vision import Camera2D
import tensorflow as tf

# Optimize TensorFlow configuration
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # Reduce logging
tf.config.run_functions_eagerly(False)

# GPU Configuration 
def setupGPU():
    
    gpus = tf.config.list_physical_devices('GPU')
    print("GPUs:", gpus)
    if gpus:
        # Enable memory growth to prevent TensorFlow from consuming all GPU memory
        for gpu in gpus:
            print("\nDetailed GPU Info:")
            print(f"Name: {gpu.name}")
            return gpu
            try:
                tf.config.experimental.set_memory_growth(gpu, True)
                print("memory thingy enabled")
            except Exception as e:
                print(f"GPU setup error: {e}")

device = setupGPU()
print(f"Model Utilizing: {device}")

# Precompute constant values
HEIGHT_CROP_START = 0.5
HEIGHT_CROP_END = 0.85
GRAYSCALE_CROP_START = 10
STEERING_CLASSES = np.array([-0.5, -0.4375, -0.375, -0.3125, -0.25, -0.1875, -0.125, -0.0625, 
                              0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375, 0.5])

myCar = QCar(readMode=0)
lidar_device = Lidar(type='RPLidar')
camera_front = Camera2D(cameraId="3", frameWidth=420, frameHeight=220, frameRate=30)

# Use numpy for more efficient calculations
CAR_PARAMS = {
    'deceleration_increment': 0.00001,
    'throttle_increment': 0.005,
    'steering_increment': 0.005,
    'max_throttle': 0.1,
    'min_throttle': -0.1,
    'max_steering': 0.5,
    'min_steering': -0.5,
}

def loadModel(filename):
    """Optimized model loading with better error handling"""

    try:

        #These paths both dont exist lol
        gpu_delegate_paths = [
            '/usr/lib/aarch64-linux-gnu/tegra/libtensorflowlite_gpu_delegate.so',
            '/usr/local/lib/python3.6/dist-packages/tensorflow/lite/experimental/delegates/gpu/libgpu_delegate.so'
        ]

        for delegate_path in gpu_delegate_paths: 
            if os.path.exists(delegate_path):
                try: 
                    #gpu_delegate = tf.lite.experimental.load_delegate(delegate_path)
                    gpu_delegate = tf.lite.GpuDelegate(delegate_path)
                    interpreter = tf.lite.Interpreter(
                        model_path = filename,
                        experimental_delegates = [gpu_delegate]
                    )
                
                    print(f"Loaded model with GPU from {delegate_path}")
                    interpreter.allocate_tensors()
                    return interpreter
                except Exception as e: 
                    print(f"Failed to load GPU from {delegate_path}")

        print("going back to CPU")
        interpreter = tf.lite.Interpreter(model_path = filename)
        interpreter.allocate_tensors()
        return interpreter
    except Exception as e: 
        print("Model loading error: {e}")
        raise


    '''
    try:
        # Attempt GPU delegate first
        gpu_delegate = tf.lite.experimental.load_delegate("libtensorflowlite_gpu_delegate.so")
        driving_model = tf.lite.Interpreter(model_path=filename, experimental_delegates=[gpu_delegate])
        print("Using GPU Delegate for TensorFlow Lite")
    except Exception as e:
        print(f"GPU delegate error: {e}. Falling back to CPU.")
        driving_model = tf.lite.Interpreter(model_path=filename)

    driving_model.allocate_tensors()
    return driving_model
    '''

def camPreview(camIDs):
    """Optimized camera preview with error handling"""
    if "front" in camIDs:
        frame = camera_front.read()
        return frame.imageData if frame is not None else None
    return None

def getModelSteering(model, input_details, output_details):
    """Optimized model steering prediction"""
    image_cap_np = camPreview(camIDs=["front"])
    if image_cap_np is None:
        return 0.0  # Default steering if no image

    height, width = image_cap_np.shape[:2]
    
    # Vectorized image preprocessing
    image_cropped = image_cap_np[
        int(height*HEIGHT_CROP_START):int(height*HEIGHT_CROP_END), :
    ]
    image_grayscale = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2GRAY)
    image_processed = image_grayscale[GRAYSCALE_CROP_START:, :][np.newaxis, :, :, np.newaxis]
    image_processed = image_processed.astype(np.float32)
    
    # Optimized model inference
    model.set_tensor(input_details[0]['index'], image_processed)
    model.invoke()
    
    predictions = model.get_tensor(output_details[0]['index'])
    return STEERING_CLASSES[np.argmax(predictions)]

def check_obstacle(lidar_device, danger_threshold=0.6):
    """Efficient obstacle detection"""
    if (lidar_device.distances is None or 
        lidar_device.angles is None or 
        lidar_device.distances.size == 0):
        return False, 0.0

    # Vectorized angle and distance filtering
    front_view_mask = (lidar_device.angles >= np.deg2rad(70)) & (lidar_device.angles <= np.deg2rad(110))
    front_distances = lidar_device.distances[front_view_mask]
    front_distances = front_distances[front_distances > 0]

    if len(front_distances) > 0 and np.any(front_distances < danger_threshold):
        # Efficient side clearance calculation
        angles = lidar_device.angles
        distances = lidar_device.distances
        
        left_view = (angles >= np.deg2rad(60)) & (angles <= np.deg2rad(90))
        right_view = (angles >= np.deg2rad(90)) & (angles <= np.deg2rad(120))
        
        left_distances = distances[left_view]
        right_distances = distances[right_view]
        
        left_distances = left_distances[~np.isnan(left_distances)]
        right_distances = right_distances[~np.isnan(right_distances)]
        
        left_clearance = np.nanmean(left_distances) if left_distances.size > 0 else np.inf
        right_clearance = np.nanmean(right_distances) if right_distances.size > 0 else np.inf
        
        return True, (left_clearance, right_clearance)
    
    return False, (0.0, 0.0)

def main():
    # Load ML model with performance optimization
    driving_model = loadModel("Models/Mar12Class.tflite")
    input_details = driving_model.get_input_details()
    output_details = driving_model.get_output_details()
    
    # Initialize state variables
    throttle = 0.0
    steering = 0.0
    LEDs = np.zeros(8, dtype=np.int8)
    
    print("Main control loop started. Press Ctrl+C to exit.")
    
    try:
        while True:
            # Read LiDAR data
            lidar_device.read()
            
            # Efficient obstacle detection
            obstacle_detected, clearances = check_obstacle(lidar_device)
            
            if obstacle_detected:
                left_clearance, right_clearance = clearances
                
                # Optimized obstacle avoidance
                if left_clearance > right_clearance + 0.1:
                    new_steering = CAR_PARAMS['max_steering']
                    throttle = 0.07
                elif right_clearance > left_clearance + 0.1:
                    new_steering = CAR_PARAMS['min_steering']
                    throttle = 0.07
                else:
                    new_steering = 0.0
                    throttle = 0.08
                
                steering = 0.7 * steering + 0.3 * new_steering
            else:
                # ML model steering
                new_steering = getModelSteering(driving_model, input_details, output_details)
                steering = 0.8 * steering + 0.3 * new_steering
                throttle = 0.075
            
            # Apply control commands
            myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        lidar_device.terminate()
        myCar.terminate()

if __name__ == '__main__':
    main()
    # setupGPU()
