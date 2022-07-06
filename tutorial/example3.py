import math
import pygame
import random

# custom libraries
import env
import sensors
import features


def randomColor():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

def main():

    featureMap = features.featuresDetection()

    environment = env.buildEnvironment('map1.png')
    originalMap = environment.map.copy()
    laser = sensors.laserSensor(200, originalMap, uncertainty=(0.5, 0.01))
    environment.map.fill((255, 255, 255))
    environment.infoMap = environment.map.copy()
    originalMap = environment.map.copy()

    running = True
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0
    
    while running:
        environment.infoMap = originalMap.copy()
        FEATURE_DETECTION = True
        BREAK_POINT_IND = 0
        ENDPOINTS = [0,0]
        sensorOn = False
        PREDICTED_POINTS_TO_DRAW = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
        if pygame.mouse.get_focused():
            sensorOn = True
        elif not pygame.mouse.get_focused():
            sensorOn = False

        if sensorOn:
            position = pygame.mouse.get_pos()
            laser.position = position
            sensorData = laser.senseObstacles()
            featureMap.laserPointsSet(sensorData)

            while BREAK_POINT_IND < (featureMap.NP - featureMap.PMIN):
                seedSeg = featureMap.seedSegmentDetection(laser.position, BREAK_POINT_IND)
                if seedSeg == False:
                    break
                else:
                    seedSegment = seedSeg[0]
                    PREDICTED_POINTS_TO_DRAW = seedSeg[1]
                    INDICES = seedSeg[2]
                    results = featureMap.seedSegmentGrowing(INDICES, BREAK_POINT_IND)
                    if results == False:
                        BREAK_POINT_IND = INDICES[1]
                        continue
                    else:
                        lineEq = results[1]
                        m, c = results[5]
                        lineSeg = results[0]
                        OUTERMOST = results[2]
                        BREAK_POINT_IND = results[3]

                        ENDPOINTS[0] = featureMap.projectionP2L(OUTERMOST[0], m, c)
                        ENDPOINTS[1] = featureMap.projectionP2L(OUTERMOST[1], m, c)

                        featureMap.FEATURES.append([[m, c], ENDPOINTS])
                        pygame.draw.line(environment.infoMap, (0, 255, 0), ENDPOINTS[0], ENDPOINTS[1], 1)
                        environment.dataStorage(sensorData)

                        featureMap.FEATURES = featureMap.lineF2P()
                        features.landmarkAssociation(featureMap.FEATURES)

            for landmark in features.LANDMARKS:
                pygame.draw.line(environment.infoMap, (0, 0, 255), landmark[1][0], landmark[1][1], 2)


        environment.map.blit(environment.infoMap, (0, 0))
        pygame.display.update()

if __name__ == "__main__":
    main()