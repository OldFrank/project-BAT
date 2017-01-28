# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 14:54:41 2016

@author: Visualize_Bat 
"""
import math
import time
import random

from Tkinter import *
import matplotlib
matplotlib.use('TkAgg')
import pylab 

class bat_visualization(object):
    def __init__(self, sidelength):
        self.top = Tk()
        self.sidelength = sidelength
        self.canvas = Canvas(self.top, bg="white", height = sidelength, width = sidelength)
        self.canvas.pack()
    
    def arraytotile(self, indexx, indexy, length): #assume a square 
        ll = self.sidelength
        a = ll/length*indexx
        b = ll/length*(indexx+1)
        c = ll - ll/length*indexy
        d = ll - ll/length*(indexy+1)
        return a, c, b, d
        
    def createcanvas(self, A):
        
        for i in range(A.shape[0]):
            for j in range(A.shape[1]):
                sl = A.shape[0]
                coord = self.arraytotile(i, j, sl)
                color = "white"
                if A[i,j] < 0.5:
                    color = "white"
                else:
                    color = "black" 
                self.canvas.create_rectangle(coord, fill = color )
    
    def batoval(self, batpos, cavelength, radius):
        scale = self.sidelength/cavelength
        a = batpos.get_x()*scale - radius*scale
        b = self.sidelength - batpos.get_y()*scale - radius*scale
        c = batpos.get_x()*scale + radius*scale
        d = self.sidelength - batpos.get_y()*scale + radius*scale
        return a, b, c, d
    
    def fovarc(self, batpos, cavelength, sense_range): 
        scale = self.sidelength/cavelength
        a = batpos.get_x()*scale - sense_range*scale
        b = self.sidelength - batpos.get_y()*scale - sense_range*scale
        c = batpos.get_x()*scale + sense_range*scale
        d = self.sidelength - batpos.get_y()*scale + sense_range*scale
        return a, b, c, d
    
    def update(self, A, batcoord, cavelength, radius, direction, fov, srange, delay = 0.1):
        self.createcanvas(A)
        coord = self.batoval(batcoord, cavelength, radius)
        fov_coord = self.fovarc(batcoord, cavelength, srange)
        self.canvas.create_oval(coord, fill="red")
        self.canvas.create_arc(fov_coord, start=direction-fov/2, extent=fov, fill="blue", stipple="gray25")
        self.top.update()
        time.sleep(delay)
        
    def done(self):
        "Indicate that the animation is done so that we allow the user to close the window."
        self.top.mainloop()

