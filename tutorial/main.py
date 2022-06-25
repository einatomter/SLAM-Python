import math
import pygame

# custom libraries
import env
import sensors

def main():
    environment = env.buildEnvironment('map1.png')
    environment.originalMap = environment.map.copy()
    laser = sensors.laserSensor(200, environment.originalMap, uncertainty=(0.5, 0.01))
    environment.map.fill((0, 0, 0))
    environment.infoMap = environment.map.copy()

    running = True
    
    while running:
        sensorOn = False
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
            environment.dataStorage(sensorData)
            environment.showSensorData()
        
        environment.map.blit(environment.infoMap, (0, 0))
        pygame.display.update()

if __name__ == "__main__":
    main()