
# import lib
import pygame
from pal.products.qcar import QCar
from pal.utilities.math import *
import numpy as np
import time
#from mini_cameras_view import camPreview
from Cameras import camPreview
import threading
import sys
import os
# create qcar
myCar = QCar(readMode=0)

def Drive():

    # init Pygame
    pygame.init()

    # setup screen size
    screen_width = 800
    screen_height = 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("QCar Manual Control")

    # set up +- increment and maximum
    deceleration_increment = .00001
    throttle_increment = 0.0001
    steering_increment = 0.005
    max_throttle = 0.1
    min_throttle = -0.1
    max_steering = 0.5
    min_steering = -0.5
    MINIMUM_THROTTLE_EPSILON = .0000001

    # default speed and direction
    throttle = 0.0
    steering = 0.0

    def stop():
        throttle = 0.0
        steering = 0.0

    # main program
    runTime = 10.0 # seconds
    t0 = time.time()
    
    # Default LED status
    LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    isReverse = False
    headlights_on = True
    s_key_pressed = False

    turn_signal_timer = 0.0
    turn_signal_interval = 0.5
    turn_signal_state = 0 
    running = True
    while running == True:
    # while time.time() - t0  < runTime :
        # t = time.time()

        #listen for pygame Events in the event queue
        for event in pygame.event.get():
            #if the window is closed, shut the car off
            if event.type == pygame.QUIT:
                running = False
            #if finger has been taken off the keys
            elif event.type == pygame.KEYUP:
                #maintain steering direction
                if event.key == pygame.K_a or  event.key == pygame.K_d:
                  steering = 0.0
                #if car is in drive and the throttle key is released
                if event.key == pygame.K_w and isReverse is False:
                  #reduce throttle amount
                  while throttle > MINIMUM_THROTTLE_EPSILON:
                    throttle -= .0000001
                  #car should come to a complete stop if it has reached minimum throttle before the throttle key is pressed again
                  throttle = 0
                #if car is in reverse and throttle key is released
                if event.key == pygame.K_w and isReverse is True:
                  #reduce throttle amount
                  while throttle < MINIMUM_THROTTLE_EPSILON:
                    throttle += .0000001
                  #car comes to complete stop upon reaching minimum throttle before the throttle key has been pressed
                  throttle = 0

        keys = pygame.key.get_pressed()

        # steering
        if keys[pygame.K_w]:
            if not isReverse:
                throttle += throttle_increment
                #prevents car from going over max throttle amount
                throttle = min(throttle, max_throttle)
                LEDs[4] = 0
            if isReverse:
                #prevents car from going under min throttle amount unless it is manually set to 0 like in deceleration algorithm
                throttle -= throttle_increment
                throttle = max(throttle, min_throttle)

        elif keys[pygame.K_s]:
            if throttle > 0 and not isReverse:
                throttle -= throttle_increment
                #prevent car from going under min throttle amount
                throttle = max(throttle, min_throttle)
                LEDs[4] = 1
                s_key_pressed = True  # Set flag when 'S' key is pressed
            elif throttle < 0 and isReverse:
                throttle += throttle_increment
                #prevent car from going under min throttle amount
                throttle = max(throttle, min_throttle)
            else:
                s_key_pressed = False  # Reset flag when 'S' key is released

        if not keys[pygame.K_s] and not s_key_pressed:
            LEDs[4] = 0

        if keys[pygame.K_r]:
            isReverse = True
            throttle = 0
            LEDs[4] = 0  # Turn off LED[4] when 'R' key is pressed
            LEDs[5] = 1
        elif keys[pygame.K_f]:
            isReverse = False
            throttle = 0
            LEDs[5] = 0

        if keys[pygame.K_d]:
            steering -= steering_increment
            #turning right is steering in the negative direction
            steering = max(steering, min_steering)
        elif keys[pygame.K_a]:
            steering += steering_increment
            #turning left is steering in the positive direction
            steering = min(steering, max_steering)
        
        if keys[pygame.K_q]:
            steering , throttle = 0.0 , 0.0
            LEDs[5] = 0
        
        if keys[pygame.K_h]:
            headlights_on= not headlights_on
            LEDs[6] = 1 if headlights_on else 0
            LEDs[7] = 1 if headlights_on else 0
        
        # update turnled timer
        turn_signal_timer += 0.01

        # based on direction and speed update led control
        if steering > 0.15:
            if turn_signal_timer >= turn_signal_interval:
                turn_signal_state = 1 - turn_signal_state
                turn_signal_timer = 0.0
            if turn_signal_state:
                LEDs[0] = 1
                LEDs[2] = 1
            else:
                LEDs[0] = 0
                LEDs[2] = 0
        elif steering < -0.15:
            if turn_signal_timer >= turn_signal_interval:
                turn_signal_state = 1 - turn_signal_state
                turn_signal_timer = 0.0
            if turn_signal_state:
                LEDs[1] = 1
                LEDs[3] = 1
            else:
                LEDs[1] = 0
                LEDs[3] = 0
        else:
            LEDs[0] = 0
            LEDs[1] = 0
            LEDs[2] = 0
            LEDs[3] = 0

        if throttle < 0:
            LEDs[5] = 1
        else:
            LEDs[5] = 0
        

        # update qcar control
        myCar.read_write_std(throttle=throttle, steering=steering, LEDs = LEDs)
        print(throttle, steering, isReverse, LEDs)

keyControl = threading.Thread(target=Drive)
cameraAccess = threading.Thread(target=camPreview,args=["front"])

try:
    keyControl.start()
    cameraAccess.start()
except KeyboardInterrupt:
        myCar.terminate()
        # exitPygame
        pygame.quit()
        sys.exit()

    
        


