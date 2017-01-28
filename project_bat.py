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
        """ 
        define equality
        """
        if self.x == other.x and self.y == other.y:
            return True 
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_dist(self, other): 
        """
        distance between two positions
        """
        x_dist = self.get_x() - other.get_x()
        y_dist = self.get_y() - other.get_y()
        return (x_dist**2 + y_dist**2)**0.5
        
    def get_corrupt_dist(self, other, sigma):
        """
        add noise to distance 
        """        
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
        """
        define what print does 
        """
        return str((self.get_x(), self.get_y()))
        
class cave(object): 
    def __init__(self, width, height, numobstacles):
        """
        initialize a cave by the number of obstacles in a defined space 
        random seed is for testing as to get the same cave everytime for now
        self.obstacles return a list of obstacles coordinates (sorted)
        """
        self.width = width 
        self.height = height
        obstacles = []
        random.seed(0) #insure the same cave every trial for now 
        for i in range(numobstacles): #generate obstacles randomly 
            x_cor = random.randint(0,width-1) 
            y_cor = random.randint(0,height-1)
            obstacles.append((x_cor,y_cor))
        obstacles.sort()
        self.obstacles = obstacles
    
    def valid_position(self, position, radius):
        """
        check if position is valid
        """
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
    def __init__(self, cave, fov, sense_range, speed, radius = 0.2):
        """
        define a bat based on the speed it goes at and the cave it is in and it's size (approximate)
        it as a circle and define its position and direction it is heading in 
        """
        self.cave = cave
        self.radius = radius 
        #initialize it with random position and direction 
        self.direction = 0 #360*random.random()
        self.fov = fov
        self.range = sense_range
        self.speed = speed
        position = Position(cave.width*random.random(),cave.height*random.random())
        #initialize bat at random (valid) position 
        while not self.cave.valid_position(position, radius):
            position = Position(cave.width*random.random(),cave.height*random.random())
        self.position = Position(0.,0.)

    def get_bat_pos(self):
        return self.position
    
    def get_bat_dir(self):
        return self.direction
    
    def set_bat_pos(self, position):
        #position is a position object
        self.position = position
        
    def set_bat_dir(self, direction):
        self.direction = direction 
    
    def move(self, dist, angle):
        """
        continue moving in the same direction unless the next position is invalid, if next position is
        invalid (running into wall of obstacle) go to a random position
        """
        current_dir = self.get_bat_dir()
        new_dir = current_dir + angle
        self.set_bat_dir(new_dir)
        current_pos = self.get_bat_pos()
        newpos = current_pos.get_new_position(new_dir, dist)
        self.set_bat_pos(newpos)
        
    def obstacle(self, angle, position, urange, add_noise = 0, sigma = 0.5):
        """
        helper functionn to sense, see if there's an obstacle at a certain distance at a certain angle
        """
        cave = self.cave
        radians = math.radians(angle)
        for dist in range(urange+1):
            #search for obstacle 
            coor_x = int(position.get_x() + math.cos(radians)*dist)
            coor_y = int(position.get_y() + math.sin(radians)*dist)
            if (coor_x, coor_y) in cave.obstacles:
                xx = coor_x
                yy = coor_y 
                if add_noise:
                    xx = random.gauss(coor_x, sigma)
                    yy = random.gauss(coor_y, sigma)
                obs_pos = Position(xx, yy)
                return [angle, obs_pos.get_dist(position)]
        return None 
    
    def sense_obstacle(self):
        """
        return a list of what each ultrasonic sensor senses 
        """ 
        fov = self.fov
        sens_result = []
        srange = self.range
        for k in range(fov):
            angle = k - fov/2
            pos = Position(self.get_bat_pos().get_x(), self.get_bat_pos().get_y())
            scan = self.obstacle(angle, pos, srange)
            if scan != None:
                sens_result.append(scan) 
        return sens_result
    
    def bat_line_follow(self, end, p_gain, d_gain, i_gain, visual, objgrid): 
        """
        PID bat linefollower
        note that the objgrid is just for the graphics
        """
        bat_pos = [self.get_bat_pos().get_x(), self.get_bat_pos().get_y()]
        start = bat_pos
        seg_vect = [end[0] - start[0], end[1] - start[1]]
        sv_mag = math.sqrt(seg_vect[0]**2 + seg_vect[1]**2) #magnitude of segment_vector 
        cul_ce = 0
        pos_vect = [bat_pos[0] - start[0], bat_pos[1] - start[1]]
        prev_ce = (seg_vect[0]*pos_vect[1] - seg_vect[1]*pos_vect[0])/sv_mag
        #dot product to see how far along along seg 
        d_prod = (seg_vect[0]*pos_vect[0] + seg_vect[1]*pos_vect[1])/(sv_mag*sv_mag)
#        visual = bat_visualize.bat_visualization(500)      
        while d_prod < 1:
            #cross product to obtain error since a x b = |a||b|sin(theta)
            c_error = (seg_vect[0]*pos_vect[1] - seg_vect[1]*pos_vect[0])/sv_mag
            cul_ce += c_error             
            turn = - (p_gain*c_error + d_gain*(c_error - prev_ce) + i_gain*cul_ce)
            self.move(self.speed, turn)
            prev_ce = c_error
            bat_pos = [self.get_bat_pos().get_x(), self.get_bat_pos().get_y()]
            visual.update(objgrid, self.position, self.cave.width, self.radius, self.direction, self.fov, self.range)
            pos_vect = [bat_pos[0] - start[0], bat_pos[1] - start[1]]
            #dot product to see how far along along seg 
            d_prod = (seg_vect[0]*pos_vect[0] + seg_vect[1]*pos_vect[1])/(sv_mag*sv_mag)           
        return True 

#cave1 = cave(10, 10, 50)
#bat1 = bat(cave1, 90, 5, 0.1)
#bat1.bat_line_follow([2,2],120,800,30)
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
