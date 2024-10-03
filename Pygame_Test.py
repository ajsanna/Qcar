#quick overview of pygame and how to use it

import pygame
pygame.init()
# this is to bring in the pygame library and initiliaze. must be done prior to anything else

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600 

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
#this creates the screen view. This is The VIEW aspect of this program 

#the following will be the game loop so that the window stays open without closing 

run = True 
while run:
  
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      run = False
    elif event.type == pygame.KEYDOWN:
      if event.key == pygame.K_a:
        print("steering left")
      elif event.key == pygame.K_d:
        print("steering right")
      elif event.key == pygame.K_s:
        print("reverse thrust")
      elif event.key == pygame.K_w:
        print("forward thrust")
      else:
        print("invalid keybind")
        
  # the inner for loop here runs over and over again looking for events (user input)
  # We have added an event listener looking for if the red exit button is clicked to terminate the program. 
  
