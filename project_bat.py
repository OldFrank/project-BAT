# -*- coding: utf-8 -*-
"""
Created on Mon Dec 26 00:04:45 2016

@author: Yun 
PROJECT BAT 
"""
import math
import random
import bat_visualize
import pylab 
import numpy as np 

class Position(object):
    """
    A Position represents a location in a two-dimensional room, where
    coordinates are given by floats (x, y).
    """
    def __init__(self, x, y):
        """
        Initializes a position with coordinates (x, y).
        """
        self.x = x
        self.y = y
    
    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True 
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_dist(self, other): 
        x_dist = self.get_x() - other.get_x()
        y_dist = self.get_y() - other.get_y()
        return (x_dist**2 + y_dist**2)**0.5
        
    def get_corrupt_dist(self, other, sigma):
        puredist = self.get_dist(other)
        noise = random.gauss(0, sigma)
        return puredist + noise
    
    def get_new_position(self, angle, speed):
        """
        Computes and returns the new Position after a single clock-tick has
        passed, with this object as the current position, and with the
        specified angle and speed.

        Does NOT test whether the returned position fits inside the room.

        angle: float representing angle in degrees, 0 <= angle < 360
        speed: positive float representing speed

        Returns: a Position object representing the new position.
        """
        old_x, old_y = self.get_x(), self.get_y()
        
        # Compute the change in position
        delta_x = speed * math.cos(math.radians(angle))
        delta_y = speed * math.sin(math.radians(angle))
        
        # Add that to the existing position
        new_x = old_x + delta_x
        new_y = old_y + delta_y
        
        return Position(new_x, new_y)
        
    def __str__(self):
        return str((self.get_x(), self.get_y()))
        
class cave(object): 
    def __init__(self, width, height, numobstacles):
        self.width = width 
        self.height = height
        obstacles = []
        random.seed(5)
        for i in range(numobstacles): 
            x_cor = random.randint(0,width-1)
            y_cor = random.randint(0,height-1)
            obstacles.append((x_cor,y_cor))
        obstacles.sort()
        self.obstacles = obstacles
    
    def valid_position(self, position, radius):
        tilex1 = int(position.get_x() - radius)
        tiley1 = int(position.get_y() - radius)
        tilex2 = int(position.get_x() + radius)
        tiley2 = int(position.get_y() + radius)
        if tilex1 > 0 and tilex2 < self.width and tiley1 > 0 and tiley2 < self.height:
            fourcor = [(tilex1, tiley1), (tilex2, tiley2), (tilex1, tiley2), (tilex2, tiley1)]
            for cor in fourcor:
                if cor in self.obstacles:
                    return False
            return True
        return False
       

class bat(object): 
    def __init__(self, cave, speed, radius = 0.25):
        self.speed = speed
        self.cave = cave
        self.radius = radius 
        #initialize it with random position and direction 
        self.direction = 0 #360*random.random()
        position = Position(cave.width*random.random(),cave.height*random.random())
        while not self.cave.valid_position(position, radius):
            position = Position(cave.width*random.random(),cave.height*random.random())
        self.position = position
        self.ultrasonic = [] #a list of ultrasonic sensors 
    
    def get_bat_pos(self):
        return self.position
    
    def get_bat_dir(self):
        return self.direction
    
    def set_bat_pos(self, position):
        #position is a position object
        self.position = position
        
    def set_bat_dir(self, direction):
        self.direction = direction 
    
    def add_ultrasonic(self, ultrason):
        self.ultrasonic.append(ultrason) #append ultrasonic (rel_pos to bat (x,y), rel_direction, range)
    
    def move(self):
        angle = self.get_bat_dir()
        speed = self.speed
        current_pos = self.get_bat_pos()
        newpos = current_pos.get_new_position(angle, speed)
        while not self.cave.valid_position(newpos, self.radius):
            direction = random.random()*360
            self.set_bat_dir(direction)
            newpos = current_pos.get_new_position(direction, speed)
        self.set_bat_pos(newpos)
        
    def obstacle(self, angle, ultrasonic_position, urange, add_noise = 0, sigma = 0.5):
        cave = self.cave
        radians = math.radians(angle)
        for dist in range(urange+1):
            #search for obstacle 
            coor_x = int(ultrasonic_position.get_x() + math.cos(radians)*dist)
            coor_y = int(ultrasonic_position.get_y() + math.sin(radians)*dist)
            if (coor_x, coor_y) in cave.obstacles:
                xx = coor_x
                yy = coor_y 
                if add_noise:
                    xx = random.gauss(coor_x, sigma)
                    yy = random.gauss(coor_y, sigma)
                obs_pos = Position(xx, yy)
                return [obs_pos, obs_pos.get_dist(ultrasonic_position), angle]
        return None 
    
    def sense_obstacle(self, addnoise = 1):
        #return a list of what each ultrasonic sensor senses 
        sens_result = []
        for sensor in self.ultrasonic:
            angle = sensor[1]
            pos = Position(self.get_bat_pos().get_x() + sensor[0][0], self.get_bat_pos().get_y() + sensor[0][1])
            urange = sensor[2]
            scan = self.obstacle(angle, pos, urange)
            sens_result.append(scan) 
        return sens_result

#cave1 = cave(10, 10, 50)
#bat1 = bat(cave1, 1)
#bat1.add_ultrasonic(((0.1,0),0,3))
#bat1.add_ultrasonic(((0,0.1),90,3))
#bat1.add_ultrasonic(((-0.1,0),180,3))
#bat1.add_ultrasonic(((0,-0.1),270,3))
#numsteps = 30
#visual = bat_visualize.bat_visualization(500)
#obstacles = cave1.obstacles
#actualobs = np.zeros((cave1.width, cave1.height))
#actualobs = pylab.matrix(actualobs)
#for obstacle in obstacles: 
#    actualobs[obstacle[0], obstacle[1]] = 1
#for n in range(numsteps):
#    bat1.move()
#    print bat1.position
#    for obstac in bat1.sense_obstacle():
#        if obstac != None:
#            print obstac[0]
#    print "*********************************"
#    visual.update(actualobs, bat1.position, 10, bat1.radius)
#visual.done()
