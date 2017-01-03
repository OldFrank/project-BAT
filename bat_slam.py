# -*- coding: utf-8 -*-
"""
Created on Wed Dec 28 10:13:31 2016

@author: Yun 
Project BATSLAM
"""
import pylab
import numpy as np 
import project_bat
from project_bat import Position 
import math 
import bat_visualize 

class bat_slam(object):
    def __init__(self, map_size, resolution=0.5):
        #resolution is basically mapping from the cave world to this world 
        #default is the 4 grids in map corresponds to one tile in the cave 
        #initialize map grid of map_size x map_size 
        self.map_grid = np.zeros((map_size, map_size))
        #the probabilities of there being an obstacle on each grid 
        self.map_grid_prob = np.zeros((map_size, map_size))
        #initialize bat in the middle of the map 
        self.bat_pos = (int(map_size/2), int(map_size/2))
        self.resolution = resolution 
        
    def pos2grid(self, position):
        resolution = self.resolution
        grid_coor_x = int(position.get_x()/resolution)
        grid_coor_y = int(position.get_y()/resolution)
        return [grid_coor_x, grid_coor_y]
        
    def bat_meas2obs_pos(self, bat_measurement):
        sensed_pos = []
        for measurement in bat_measurement:
            try: 
                sensed_pos.append(measurement[0])
            except TypeError:
                pass
        return sensed_pos

    def interpolate(self, bat_measurement, threshold=2):
        #interpolate a line between two sened_positions if the distance between is less than threshold
        #note threshold is in caveworld units not the number of grids 
        #give all sensed girds a 0.2 probability and all interpolated a 0.1 probability 
        sensed_positions = self.bat_meas2obs_pos(bat_measurement)        
        addition_array = np.zeros(self.map_grid.shape)                
        for position in sensed_positions:
            #"color" in grid
            grid = self.pos2grid(position)
            addition_array[grid[0]][grid[1]] += 0.2
        for k in range(len(sensed_positions)):
            for pos in sensed_positions:
                if sensed_positions[k] != pos:
                    if sensed_positions[k].get_dist(pos) < threshold:
                        start = self.pos2grid(sensed_positions[k])
                        end = self.pos2grid(pos)
                        #"Color" in the connected grids 
                        while start != end:
                            if start[0] != end[0]:
                                start[0] += (end[0] - start[0])/(abs(end[0] - start[0]))
                            if addition_array[start[0]][start[1]] == 0:
                                addition_array[start[0]][start[1]] += 0.1
                            if start[1] != end[1]:
                                start[1] += (end[1] - start[1])/(abs(end[1] - start[1]))
                            if addition_array[start[0]][start[1]] == 0:
                                addition_array[start[0]][start[1]] += 0.1
        #add the probabilities onto the map                     
        self.map_grid_prob += addition_array
        for i in range(self.map_grid_prob.shape[0]):
            for j in range(self.map_grid_prob.shape[1]):
                if self.map_grid_prob[i][j] > 1.0:
                    self.map_grid_prob[i][j] = 1.0 #probability can't exceed one
     
    def update_map_grid(self):
        for i in range(self.map_grid_prob.shape[0]):
            for j in range(self.map_grid_prob.shape[1]):
                if self.map_grid_prob[i][j] == 1.0:
                    self.map_grid[i][j] = 1.0 
                elif self.map_grid_prob[i][j] > 0.5:
                    self.map_grid[i][j] = 0.5
                else: 
                    self.map_grid[i][j] = 0.0
        
        
    def kalman_filter(self, x, P, u, state_trans, measurements, H, R):              
        # prediction
        F = state_trans
        I = np.identity(P.size**0.5)
        x = (F * x) + u
        P = F * P * F.transpose()
        
        # measurement update
        Z = measurements
        y = Z - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S**(-1)
        x = x + (K * y)
        P = (I - (K * H)) * P
        return x, P 
    
    def bat_pos_from_obs(self, obs_position, dist, angle):
        #using the sensed obstacle position. infer back to bat position 
        rad = math.radians(angle)
        bat_pos_x = obs_position.get_x() - dist*math.cos(rad)
        bat_pos_y = obs_position.get_y() - dist*math.sin(rad)
        return (bat_pos_x, bat_pos_y)
    
    def takeoutNone(self, bat_meas):
        validmeas = []
        for measu in bat_meas:
            if measu != None:
                validmeas.append(measu)
        return validmeas 
        
    def ultrarray2meas(self, bat_meas):
        bat_measurement = self.takeoutNone(bat_meas)
        newmean = self.bat_pos_from_obs(bat_measurement[0][0], bat_measurement[0][1], bat_measurement[0][2])
        ref_grid = self.pos2grid(bat_measurement[0][0])
        ref1 = self.map_grid[ref_grid[0]][ref_grid[1]]
        #validity of measurement related to the probability of the measured obstacle 
        if ref1 != 0: 
            newstd = 1/ref1
        else: 
            newstd = 1000
        for i in range(1,len(bat_measurement)): #here is assuming each measurement is Gaussain 
            mean = self.bat_pos_from_obs(bat_measurement[i][0], bat_measurement[i][1], bat_measurement[i][2])
            obst_grid = self.pos2grid(bat_measurement[i][0])
            ref = self.map_grid[obst_grid[0]][obst_grid[1]]
            if ref != 0:             
                std = 1/ref
            else:
                std = 1000
            #use the fact that when multiplying Gaussians you get a new Gaussian with mean: 
            #(mean1*var2+mean2*var1)/(var1+var2) and new standard deviation: 
            #std1*std2/(var1+var2)
            newmean_x = (newmean[0]*std*std + mean[0]*newstd*newstd)/(std*std + newstd*newstd)
            newmean_y = (newmean[1]*std*std + mean[1]*newstd*newstd)/(std*std + newstd*newstd)
            newmean = (newmean_x , newmean_y)
            newstd = newstd*std/(std*std + newstd*newstd)
        meas = pylab.matrix([[newmean[0]],[newmean[1]]])
        noise = pylab.matrix([[newstd, 0],[0, newstd]])
        return meas, noise

    def linear_slam(self, prior, P, sense, dt=0.2):
        #state vector = [x x_dot y y_dot]^T
        state_trans = pylab.matrix([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
        u = 0
        H = pylab.matrix('1 0 0 0 ; 0 0 1 0') #mapping state to measurement space 
        R = pylab.matrix('10 0 ; 0 10')
        newstate, newcovar = self.kalman_filter(prior, P, u, state_trans, sense, H, R)
        return newstate, newcovar
        
        
P = pylab.array([[1,0,0,0],[0,10,0,0],[0,0,1,0],[0,0,0,10]])            

cave1 = project_bat.cave(5,5,10)
bat1 = project_bat.bat(cave1,1)
bat1.add_ultrasonic(((0.1,0),-20,3))
bat1.add_ultrasonic(((0.1,0),-10,3))
bat1.add_ultrasonic(((0.1,0),0,3))
bat1.add_ultrasonic(((0.1,0),10,3))
bat1.add_ultrasonic(((0.1,0),20,3))
batmeas = bat1.sense_obstacle()
batslam1 = bat_slam(20)
batslam1.interpolate(batmeas)
seeslam = bat_visualize.bat_visualization(500)
seeactual = bat_visualize.bat_visualization(500)
obstacles = cave1.obstacles
actualobs = np.zeros((cave1.width, cave1.height))
actualobs = pylab.matrix(actualobs)
for obstacle in obstacles: 
    actualobs[obstacle[0], obstacle[1]] = 1
meas = batslam1.ultrarray2meas(batmeas)
numsteps = 50
prior = pylab.matrix(pylab.array([[meas[0][0,0]],[0],[meas[0][1,0]],[0]]))
batslam1.update_map_grid()
updatecounter = 0
for step in range(numsteps):
    bat1.move()
    bm = bat1.sense_obstacle()
    batslam1.interpolate(bm)
    updatecounter += 1
    if updatecounter == 5:
        batslam1.update_map_grid()
        updatecounter = 0
    meas = batslam1.ultrarray2meas(bm)
    newx, newP = batslam1.linear_slam(prior, P, meas[0], dt=0.2)
    batpos = Position(newx[0,0], newx[2,0])
    seeslam.update(batslam1.map_grid, batpos, 5, bat1.radius)
    seeactual.update(actualobs, bat1.position, 5, bat1.radius)
    print newx
    print bat1.position
    prior = newx
    P = newP 
seeslam.done()

#compare to actual 
       
        
    
