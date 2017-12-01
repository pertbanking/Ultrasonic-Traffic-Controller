import math
import pygame
from pygame.locals import *

def getDist(x1, y1, x2, y2):
    return math.sqrt(math.pow((x1-x2), 2) + math.pow((y1-y2), 2))

def rotX(right, curx, cury):
    if right:
        return curx*0.866 - cury *0.5
    else:
        return curx*0.866 + cury * 0.5
def rotY(right, curx, cury):
    if right:
        return curx * 0.5 + cury * 0.866
    else:
        return curx * -0.5 + cury * 0.866


pygame.init()
screen = pygame.display.set_mode((600, 600), 0, 32)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)

xmin = 0
xmax = 600
ymin = 0
ymax = 600
rad = 5

m_x = xmax/2
m_y = ymax/2

a_x = xmin + 1;
a_y = ymin + 1;

heading_x = 10
heading_y = 10

pygame.draw.circle(screen, red, (a_x, a_y), rad, 1)

while True:
        moved = False
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    m_x -= 10
                    moved = True
                elif event.key == pygame.K_RIGHT:
                    m_x += 10
                    moved = True
                elif event.key == pygame.K_DOWN:
                    m_y += 10
                    moved = True
                elif event.key == pygame.K_UP:
                    m_y -= 10
                    moved = True

        if moved:
            if(a_x < xmax and a_y < ymax):

                tmp_x = heading_x
                tmp_y = heading_y
                rotLeftX = rotX(False, tmp_x, tmp_y)
                rotLeftY = rotY(False, tmp_x, tmp_y)

                rotRightX = rotX(True, tmp_x, tmp_y)
                rotRightY = rotY(True, tmp_x, tmp_y)

                # check if you're in proximity
                if(getDist(a_x, a_y, m_x, m_y) <= 80):
                    print("within")
                    distLeft = getDist((a_x + rotLeftX), (a_y + rotLeftY), m_x, m_y)
                    distRight = getDist(a_x + rotRightX, a_y + rotRightY, m_x, m_y)

                    if(distLeft > distRight):
                        heading_x = rotLeftX
                        heading_y = rotLeftY
                    else:
                        heading_x = rotRightX
                        heading_y = rotRightY
                else:
                    print("not within")
                    # calculate the slope to the destination
                    if a_y < a_x: # if it's above the line from corner to corner
                        print("above")
                        heading_x = rotRightX
                        heading_y = rotRightY
                        #if heading_y > 0: #heading down
                        #    print("rot right")
                        #    heading_x = rotRightX
                        #    heading_y = rotRightY
                        #else:
                        #    print("rot left")
                        #    heading_x = rotLeftX
                        #    heading_y = rotLeftY
                    else:
                        heading_x = rotLeftX
                        heading_y = rotLeftY
                        #print("above")
                        #if heading_y < 0: # heading down
                        #
                        #    heading_x = rotRightX
                        #    heading_y = rotRightY
                        #else:
                        #    heading_x = rotLeftX
                        #    heading_y = rotLeftY
            a_x += int(heading_x)
            a_y += int(heading_y)
            print("x: " + str(a_x))
            print("y: " + str(a_y))
            screen.fill(white)
            pygame.draw.circle(screen, red, (a_x, a_y), rad, 1)
            pygame.draw.circle(screen, green, (m_x, m_y), rad, 1)
            pygame.display.update()
