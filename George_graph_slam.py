# -*- coding: utf-8 -*-
"""
Created on Fri Feb 03 15:48:26 2017
SLAM for George 
@author: Yun Chang 
"""
import math 
import pylab 
import numpy as np 
import random 
import bat_visualize as bv
from Tkinter import *

def expand_matrix(matrix, dimx, dimy, list1, list2 = []):
    """
    expanding an old matrix to a new matrix 
    for example: a = Matrix([[1,2],[3,4]])
    b = expand_matrix(a, 3, 3, [0,2],[0,1]) 
    b = Matrix([[1,2,0],[0,0,0],[3,4,0]])
    the two lists tells you where to put the original 
    """
    matrix_dim = matrix.shape
    if list2 == []:
        list2 = list1
    if len(list1) > matrix_dim[0] or len(list2) > matrix_dim[1]:
        raise ValueError, "list invalid in expand()"

    new = np.zeros((dimx, dimy))
    
    for i in range(len(list1)):
        for j in range(len(list2)):
            new[list1[i], list2[j]] = matrix[i,j]
    return pylab.matrix(new)

def take_matrix(matrix, list1, list2 = []):
    """
    taking a matrix from a bigger matrix 
    for example: a = Matrix([[1,2,3],[4,5,6],[7,8,9]])
    b = take_matrix(a, [0,2],[0,1]) 
    b = Matrix([[1,2],[7,8]])
    """
    matrix_dim = matrix.shape
    if list2 == []:
        list2 = list1
    if len(list1) > matrix_dim[0] or len(list2) > matrix_dim[1]:
        raise ValueError, "list invalid in take()"

    new = np.zeros((len(list1),len(list2)))
    for i in range(len(list1)):
        for j in range(len(list2)):
            new[i,j] = matrix[list1[i],list2[j]]
    return pylab.matrix(new)

"""
Attempt 2D slam given the yaw and linear accelerations of the quadcopter (George)
What's given: 
data from 3 ultrasonic sensors, angular positions and linear accelerations from 
the MPU 6050 Gyroscope 
Thinking about motion I need the dx and dy of each step this can be obtained using
the accelerations, the previous two positions, and the yaw, and keeping track of 
time/intervals
Upons obtaining dx and dy, one can use Graph SLAM to obtain landmark positions and 
current positions. 
Attempted algorithm as such 
in a loop: 
1. take data and turn it into dx, dy (motion from last known pose) and the measurments
(measurements for each landmark will be presented in a [landmark_number, x_away, y_away])
2. If there is measurement corresponding to a previously unknown landmark, add this new 
landmark
3. Enter everything into Omega and Xi matrices and perform graph slam (note that the initial
pose is just taken from last run through loop) 
4. Get the newly calculated robot pose and landmark pose. If robot pose is close to the edge,
initialize new grid 
END
data = [[a_x, a_y, a_z],[yaw,pitch,roll],[[x1,y1],[x2,y2],[x3,y3]], time] #the x1 y1 corresponds
to the x dist and y dist away from the quad of the obstacle measured by ultrasonic sensor 1
#time is the time since the last package (measurement) especially the IMU measurement 
DUE to the nature of graph slam, it will take a data_packet at a time (since it makes zero sense 
to perform graph slam on one single piece of data) 
"""
class graph_slam(object):
    def __init__(self, world_size, data_packet, motion_noise, measurement_noise):
        self.world_size = world_size
        self.data_packet = data_packet
        #note a data_packet is a list [data1, data2, ...] 
        #format of each data is as described above 
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        #this noise depends, of course on how much you can trust your sensor values, in
        #our case this is the ultrasonic sensors and the IMU
        self.landmark = {} #store landmarks as tiles 
        #idea is to keep units consistent from robot world, centimeters in my case 
        self.veloc = [0,0]
        self.prev_a = [0,0]
        """
        note that velocity was calculated from prev_x and prev_y and time_int, which is 
        the time interval between previous and now.  
        note to my future self time should be measured from when the IMU measurements were
        taken. So perhaps implement time measurement in Arduino? Would be so much easier if 
        there was a ROS package. This is so inprecise *sigh
        """ 
    
    def getmotion(self, data):
        """
        calculate dx, dy from stored value and new data 
        """
        init_veloc = self.veloc
        timeInt = data[-1]
        a_x = data[0][0] #+ self.prev_a[0])/2.
        a_y = data[0][1] #+ self.prev_a[1])/2.
        newveloc = [init_veloc[0] + a_x*timeInt, init_veloc[1] + a_y*timeInt]
#        displ = [init_veloc[0]*timeInt + a_x*(.5)*timeInt**2, init_veloc[1]*timeInt + a_y*(.5)*timeInt**2]
        self.veloc = newveloc
        displ = [newveloc[0]*timeInt, newveloc[1]*timeInt]
        #so I just want dx and dy so multiply by time 
        return displ
    
    """ 
    i am an idiot i need to initiate a list first or nothing in for loop
    """
    
    def update_lm(self, data, position):
        """
        see of there is a new landmark, and add it to the landmarks if there is 
        a landmark is new if it is not close to an original landmark, the closeness 
        is defined by the threshold in the function argument 
        if too close to an original: update the value 
        """
        sensor_data = data[2]
        newlandmarks = self.landmark.copy()
        for meas in sensor_data:
            
            if meas != None:
                mes = [meas[0] + position[0], meas[1] + position[1]]
                createNew = True
                for k in self.landmark:
                    dist = math.sqrt((mes[0] - self.landmark[k][0])**2 + (mes[1] - self.landmark[k][1])**2)                     
                    if dist < 1/self.measurement_noise:
                        newlandmarks[k] = mes
                        createNew = False
                if createNew:
                    #note that landmark numbering starts with 0 consistent with python practices 
                    landmark_num = len(newlandmarks)
                    newlandmarks[landmark_num] = mes #add new landmark to landmarks 
        self.landmark = newlandmarks
        return True 
    
    def initialize_lm(self,data,position):
        """
        need to start with some sort of dictionary, initialize with first data in packet
        """
        sensor_data = data[2]
        counter = 0
        for i in range(len(sensor_data)):
            if sensor_data[i] != None:
                self.landmark[counter] = [sensor_data[i][0] + position[0],sensor_data[i][1] + position[1]]
                counter += 1
        return True 
    
    def graphy_slamy(self):
        """
        so, by this time the self.x and self.y values were the previous values which is 
        exactly what we want. Motion was calculated from getmotion method 
        """
        x = self.world_size/2.
        y = self.world_size/2.#initialize in the middle of the world
        #first get an idea of the size of omega 
        self.initialize_lm(self.data_packet[0],[x,y]) #process has not started yet, just "getting a feel" 
        num_landmarks = len(self.landmark)
        dim = (1 + num_landmarks)*2
        omega = pylab.matrix(np.zeros((dim, dim)))
        omega[0,0] = 1.
        omega[1,1] = 1. 
        xi = pylab.matrix(np.zeros((dim,1)))
        xi[0,0] = x
        xi[1,0] = y 

        for step in range(len(self.data_packet)):
            data = self.data_packet[step]
            #first update landmark 
            prev_num_landmarks = len(self.landmark)#need to modify omega if num_landmark changes 
            self.update_lm(data,[x,y]) 
            if prev_num_landmarks < len(self.landmark): #when there are new landmarks... 
                newdim = 2*(1+len(self.landmark))
                omega = expand_matrix(omega,newdim,newdim,range(2*(1+prev_num_landmarks)))
                xi = expand_matrix(xi,newdim,1,range(2*(1+prev_num_landmarks)),[0])
                dim = newdim
                #expand omega to accomodate new landmarks 
            #next get dat motion going oh yeah 
            step_motion = self.getmotion(data) #obtain [dx, dy]
            #here we start to standard graph slam procedure and update omega
            for i in  self.landmark:
                lx = self.landmark[i][0] - x
                ly = self.landmark[i][1] - y
                lcoor = [lx, ly] 
                for k in range(2):
                    omega[k,k] += 1./self.measurement_noise
                    omega[k,2*(1+i)+k] += -1./self.measurement_noise
                    xi[k,0] += -lcoor[k]/self.measurement_noise
                    omega[2*(1+i)+k,2*(1+i)+k] += 1./self.measurement_noise
                    omega[2*(1+i)+k,k] += -1./self.measurement_noise
                    xi[2*(1+i)+k,0] += lcoor[k]/self.measurement_noise 
                    #last 6 lines are updating measurement information             
            #now here we expand omega and xi to accomodate for the next pose 
            omega = expand_matrix(omega, dim+2, dim+2, [0,1]+range(4, dim+2))
            xi = expand_matrix(xi, dim+2, 1, [0,1]+range(4, dim+2), [0])
            #expand leaving third and forth row for expansion 
            #now we deal with motion 
            for j in range(2):
                omega[j,j] += 1./self.motion_noise
                omega[j,j+2] += -1./self.motion_noise
                xi[j,0] += -step_motion[j]/self.motion_noise
                omega[j+2,j+2] += 1./self.motion_noise
                omega[j+2,j] += -1./self.motion_noise
                xi[j+2,0] += step_motion[j]/self.motion_noise
                #and that completes the motion update
            #putting it together 
            A = take_matrix(omega,[0,1],range(2, dim + 2))
            B = take_matrix(omega,[0,1])
            C = take_matrix(xi,[0,1],[0])
            omega = take_matrix(omega,range(2,dim + 2)) - A.transpose()*B.getI()*A
            xi = take_matrix(xi,range(2,dim+2),[0]) - A.transpose()*B.getI()*C
            mu = omega.getI()*xi 
            self.prev_a = [data[0][0], data[0][1]]
            x = mu[0,0]
            y = mu[1,0]

        return mu
        
"""
make data for testing
try first without acceleration but go really slow constant velocity 
"""
def make_landmarks(world_size, num_landmarks):
    landmarks = []
    for i in range(num_landmarks):
        coord = [round(random.random()*world_size), round(random.random()*world_size)]
        if coord != [world_size/2, world_size/2]:
            landmarks.append(coord)
    return landmarks

def ultrasonic_sens(angle, position, landmarks):
    rangelist = range(100)
    anglelist = [angle-math.radians(5), angle-math.radians(3), angle, angle+math.radians(3), angle+math.radians(5)]
    for ang in anglelist:
        for dist in rangelist:
            tile = [round(position[0] + dist*math.cos(ang)), round(position[1] + dist*math.sin(ang))]        
            if tile in landmarks:
                return [round(dist*math.cos(angle)), round(dist*math.sin(angle))]
        
def make_data(N, num_landmarks, world_size, speed):
    pos_x = world_size/2.
    pos_y = world_size/2.
    poslist = [[pos_x,pos_y]]
    vel_x = 0.
    vel_y = 0.
    yaw = 0.
    timeInt = 0.1
    data_pack = []
#    landmarks = make_landmarks(world_size, num_landmarks)
    landmarks = [[i,60] for i in range(100)]
    landmarks = landmarks + [[i,40] for i in range(100)]
    while len(data_pack) < N:
        #sense (so i only have three sensors -- gotta do this sensing business)
        # assume one sensor points straight ahead, one to the left 45 degrees and one to the right
        ultrasonic_angles = [yaw-math.radians(45), yaw, yaw+math.radians(45)]
        sense = []
        for sensor in ultrasonic_angles:
            sense.append(ultrasonic_sens(sensor,[pos_x,pos_y],landmarks))
        #move 
        posValid = False
        newpos = [speed*timeInt*math.cos(yaw) + pos_x,speed*timeInt*math.sin(yaw) + pos_y]
        if newpos[0] < world_size and newpos[0] >= 0 and newpos[1] < world_size and newpos[1] >= 0:
            if [round(newpos[0]), round(newpos[1])] not in landmarks:
                posValid = True
        while not posValid:
            yaw = random.random()
            newpos = [speed*timeInt*math.cos(yaw),speed*timeInt*math.sin(yaw)]
            if newpos[0] < world_size and newpos[0] >= 0 and newpos[1] < world_size and newpos[1] >= 0:
                if [round(newpos[0]), round(newpos[1])] not in landmarks:
                    posValid = True
        newvel_x = (newpos[0] - pos_x)/timeInt
        newvel_y = (newpos[1] - pos_y)/timeInt
        accel = [(newvel_x-vel_x)/timeInt, (newvel_y-vel_y)/timeInt, 0] #[a_x, a_y, a_z]
        vel_x = newvel_x
        vel_y = newvel_y
        [pos_x, pos_y] = newpos
        #make data: data = [[a_x, a_y, a_z],[yaw,pitch,roll],[[x1,y1],[x2,y2],[x3,y3]], time]
        data = [accel,[yaw,0,0],sense,timeInt]
        data_pack.append(data)
        poslist.append(newpos)
    return [data_pack, [pos_x,pos_y],landmarks,poslist]
         
        
"""
test with data that has mostly 0 acceleration 
"""

a = make_data(100, 70, 100, 10)
a_viz = bv.bat_visualization(500)
r_viz = bv.bat_visualization(500)
lnd_array = np.zeros((100,100))
for ldn in a[2]:
    try:
        lnd_array[ldn[0],ldn[1]] = 1.0
    except IndexError:
        pass

gs = graph_slam(100,a[0],0.1,1)
result = gs.graphy_slamy()
pred_array = np.zeros((100,100))
for i in range(1,len(result)/2):
    try: 
        pred_array[round(result[i*2]),round(result[2*i+1])] = 1.0
    except IndexError:
        pass
a_viz.createcanvas(lnd_array)
r_viz.createcanvas(pred_array)
a_viz.top.update()
r_viz.top.update()
a_viz.done()
r_viz.done()
print "result: ",result
print "final pos: ",a[1]
print "landmarks: ",a[2]
print "poslist: ",a[3]