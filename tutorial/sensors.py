import math
import pygame
import numpy as np

def uncertaintyAdd(distance, angle, sigma):
    mean = np.array([distance, angle])
    covariance = np.diag(sigma ** 2)
    distance, angle = np.random.multivariate_normal(mean, covariance)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]

class laserSensor:
    def __init__(self, range, map, uncertainty):
        self.range = range
        self.speed = 4 # rounds per second
        self.sigma = np.array([uncertainty[0], uncertainty[1]])
        self.position = (0,0)
        self.map = map
        self.mapW, self.mapH = pygame.display.get_surface().get_size()
        self.sensedObstacles = []
    
    # euclidean distance formula
    def distance(self, obstaclePosition):
        px = (obstaclePosition[0] - self.position[0]) ** 2
        py = (obstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(px+py)

    # main sensor functionality
    def senseObstacles(self):
        data = []
        x1, y1 = self.position[0], self.position[1]

        for angle in np.linspace(0, 2*math.pi, 60, False):
            # x, y sensor distance from robot position
            x2 = x1 + self.range * math.cos(angle)
            y2 = y1 - self.range * math.sin(angle)

            # sample inbetween 0 - 100% of max sensor range
            for i in range(0, 100):
                u = i/100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))

                # ensure it is within map boundaries
                if 0 < x < self.mapW and 0 < y < self.mapH:
                    color = self.map.get_at((x,y))

                    # if sample is solid (black)
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance((x,y))
                        output = uncertaintyAdd(distance, angle, self.sigma)
                        output.append(self.position) # format: (distance, angle, (posx, posy))

                        # store the measurements
                        data.append(output)
                        break
            
        if len(data) > 0:
            return data
        else:
            return False