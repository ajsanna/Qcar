
import argparse
#from pal.utilities.vision import Camera2D
#from pal.products.qcar import QCarRealSense
import threading
import sys
import socket
import os
from datetime import datetime
import numpy as np

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'  # or any {'0', '1', '2'}

import cv2
import payload
stopthread = False
PORT = 38821  # Port to listen on (non-privileged ports are > 1023)

def drive():
    print("Driving Starting...")
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('', PORT))
        global stopthread

        while True:
            if stopthread:
                break

            data = s.recvfrom(100)[0].decode('utf-8')
            if not data:
                pass

            packet = payload.payload_handler(data)
            buffer = []
            try: 
                print(buffer)
            except Exception as e:
                print("Invalid Packet Size")
                print(e)

    print("Terminated Driving")
    
    
    



def main():
  drive()

if __name__ == '__main__':
    main()