import pygame
import sys
import numpy as np
from numpy import cos, sin, pi
from scipy.optimize import minimize

# Initialize pygame
pygame.init()
desiredX = 40
desiredY = 20
desiredTheta = 0


# Set up display dimensions
canvas_width = 800
canvas_height = 600
m = 5*16.000
n = 5*9.400
q = 5*5.0
degToRad = pi/180

def dist2(x0,y0, x1, y1):
    return (x0-x1)**2 + (y0-y1)**2



# Create the canvas
canvas = pygame.display.set_mode((canvas_width, canvas_height))

def transformCoord(x, y):
    translateX = canvas_width/2
    translateY = canvas_height/2
    newX = x + translateX
    newY = -y + translateY
    return newX, newY

def circle (x, y, radius, stroke = (0, 0, 0)):
    newX, newY = transformCoord(x,y)
    pygame.draw.circle(canvas, stroke, (newX, newY), radius, 2)

def line (xStart, yStart, xEnd, yEnd, stroke = (0, 0, 0)):
    nxStart, nyStart = transformCoord(xStart, yStart)
    nxEnd, nyEnd = transformCoord(xEnd, yEnd)
    pygame.draw.line(canvas, stroke, (nxStart, nyStart), (nxEnd, nyEnd), 5)

def forwardKinematics(x):
  alfa, beta, theta = x[0], x[1], x[2]
#   desiredX = 40
#   desiredY = 20
  stroke = (255/2, 255/2, 0)
  circle(desiredX, desiredY, 10, stroke)
  
  x0 = 0
  y0 = 0
  xm = x0 + m * sin(alfa)
  ym = y0 + m * cos(alfa)
  xn = xm + n * sin(alfa + beta)
  yn = ym + n * cos(alfa + beta)
  xq = xn + q * sin(alfa + beta + theta)
  yq = yn + q * cos(alfa + beta + theta)
  thetaG = alfa + beta +theta - pi/2
  dist = (xq - desiredX)**2 + (yq - desiredY)**2  + (thetaG - desiredTheta)**2
  return dist

def drawIK(x):
  alfa, beta, theta = x[0], x[1], x[2]
  desiredX = 40
  desiredY = 20
  stroke = (255/2, 255/2, 0)
  circle(desiredX, desiredY, 10, stroke)
  
  x0 = 0
  y0 = 0
  xm = x0 + m * sin(alfa)
  ym = y0 + m * cos(alfa)
  xn = xm + n * sin(alfa + beta)
  yn = ym + n * cos(alfa + beta)
  xq = xn + q * sin(alfa + beta + theta)
  yq = yn + q * cos(alfa + beta + theta)
  stroke = (255, 0, 0)
  circle(x0,y0,10, stroke)
  line(x0, y0, xm, ym, stroke)
  stroke = (0, 255, 0)
  circle(xm,ym,10, stroke)
  line(xm, ym, xn, yn, stroke)
  stroke = (0, 0, 255)
  circle(xn,yn,10, stroke)
  line(xn, yn, xq, yq, stroke)
  stroke = (0, 0, 0)
  circle(xq,yq, 20, stroke)
  thetaG = alfa + beta + theta - pi/2
  dist = (xq - desiredX)**2 + (yq - desiredY)**2
  return dist


# Set the title of the window
pygame.display.set_caption("Line Drawing")

# Define colors
white = (255, 255, 255)

# Line coordinates (start and end points)
line_start = (100, 100)
line_end = (700, 400)

# Set the line color
line_color = (0, 0, 0)  # Black
red = (255, 0, 0)
x0 = np.array([0,0,0])
# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the canvas with white color
    canvas.fill(white)
    circle(0,0, m+n+q,(0,0,0))
    circle(0,0, m+n,(0,0,0))
    circle(0,0, m-n,(0,0,0))
    circle(0,0, m-n+q,(0,0,0))
    circle(0,0, m+n - q,(0,0,0))



    # Draw the line on the canvas
    # pygame.draw.line(canvas, line_color, line_start, line_end, 5)  # 5 is the line thickness
    # circle(0, 0, 10)
    # circle(0, 200, 10)
    # line(0, 0, 0, 200)
    desiredX, desiredY = pygame.mouse.get_pos()
    desiredX-=canvas_width/2
    desiredY-=canvas_height/2
    desiredY*=-1
    mouse_buttons = pygame.mouse.get_pressed()
    font = pygame.font.Font(None, 36)
    text = font.render(str(round(desiredTheta/degToRad)), True, (0,0,0))
    canvas.blit(text, (10, 10))
    # Check if the left mouse button is pressed
    if mouse_buttons[0]:  # Index 0 represents the left button
        desiredTheta+=1*degToRad
    if mouse_buttons[2]:
        desiredTheta-=1*degToRad
    desiredTheta = desiredTheta%(2*pi)
    
    res = minimize(forwardKinematics, x0, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})
    print("RES: ",res.x)
    alfa = -45*degToRad
    beta = 90*degToRad
    thetaL = 90*degToRad
    x0 = res.x
    #desiredTheta = alfa + beta + thetaL - pi/2
    # x0[-1] = desiredTheta - x0[0] - x0[1] +pi/2
    drawIK(res.x)
    text = font.render(str(round(forwardKinematics(res.x),2)), True, (0,0,0))
    canvas.blit(text, (10, 40))

    # Update the display
    pygame.display.flip()

# Quit pygame
pygame.quit()

# Exit the program
sys.exit()