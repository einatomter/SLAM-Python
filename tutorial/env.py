import math
import pygame

class buildEnvironment:
    def __init__(self, imageName):
        pygame.init()
        self.pointCloud = [] # data of all points found
        self.externalMap = pygame.image.load(imageName)

        self.mapH = self.externalMap.get_height()
        self.mapW = self.externalMap.get_width()

        print(f'map height: {self.mapH}')
        print(f'map width: {self.mapW}')

        self.mapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.mapWindowName)
        
        self.map = pygame.display.set_mode((self.mapW, self.mapH))
        self.map.blit(self.externalMap, (0,0))

        # colors
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    # r, theta to x, y
    def ad2pos(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = -distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))

    # check if point should be added to data
    def dataStorage(self, data):
        print(len(self.pointCloud))
        for element in data:
            point = self.ad2pos(element[0], element[1], element[2])
            if point not in self.pointCloud:
                self.pointCloud.append(point)

    # populate map with pointcloud data
    def showSensorData(self):
        self.infoMap = self.map.copy()
        for point in self.pointCloud:
            self.infoMap.set_at((int(point[0]), int(point[1])), self.red)