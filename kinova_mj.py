# -*- coding: utf-8 -*-
"""
Created on Mon May 28 10:37:34 2018

@author: jack

"""

from mujoco_py import *
from PID import *
from Rotations import *
from save_state import *
from sim_selector import *
import time
import csv
import math

       
class MUJOCO(object):
    def __init__(self):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._current_steps = 0
        self._timestep = 0.0001
        self._experiment = None
        self._mujoco = 0
        self._simulator = "Mujoco"

        self._model = load_model_from_path("kinova_description/urdf/m1n6s300.xml")
        self._sim = MjSim(self._model)
        self._viewer = MjViewer(self._sim)
        self._sim.model.opt.timestep = self._timestep
        
        #Sim Switching
        self._next_sim = None
        self._switch_sim = False

    def set_experiment(self,experiment):
        self._experiment = experiment
    
    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep)
        
    def set_current_steps(self,time):
        self._current_steps = time/self._timestep
        
    def run_mujoco(self,time,save):
        
        save.restore_mujoco_states(self._sim)
        save = SAVE_STATE()
        self.set_num_steps()
        self.set_current_steps(time)
        
        for simStep in range(round(self._current_steps),self._num_steps):  
            
            self._pid = set_positions(self._timestep, self._pid,self._experiment,self._simulator,simStep)
                
            if simStep % 500 == 0:
                for jointNum in range(6):
                    self._theta[jointNum] = self._sim.data.sensordata[jointNum]
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    self._sim.data.ctrl[jointNum] = self._linearVelocity[jointNum]
                self._positions.append([self._sim.data.get_body_xpos('m1n6s300_link_6')[0],self._sim.data.get_body_xpos('m1n6s300_link_6')[1],self._sim.data.get_body_xpos('m1n6s300_link_6')[2],self._sim.data.get_body_xquat('m1n6s300_link_6')[0],self._sim.data.get_body_xquat('m1n6s300_link_6')[1],self._sim.data.get_body_xquat('m1n6s300_link_6')[2],self._sim.data.get_body_xquat('m1n6s300_link_6')[3],self._sim.data.get_body_xpos('cube')[0],self._sim.data.get_body_xpos('cube')[1],self._sim.data.get_body_xpos('cube')[2],self._sim.data.get_body_xquat('cube')[0],self._sim.data.get_body_xquat('cube')[1],self._sim.data.get_body_xquat('cube')[2],self._sim.data.get_body_xquat('cube')[3]])
            
            self._sim.step()
            self._viewer.render()
            
            self._next_sim,self._switch_sim = simulator_selector([],self._mujoco)
            
            if self._switch_sim or simStep == self._num_steps:
                save.save_mujoco_states(self._sim)
                return simStep*self._timestep, self._positions, self._next_sim, save
