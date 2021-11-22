# -*- coding: utf-8 -*-
"""
Created on Tue May  1 17:42:03 2018

@author: jack
"""

import pybullet as p
import pybullet_data
from PID import *
from Rotations import *
from save_state import *
from sim_selector import *
from time import sleep
import csv
import math


class PYBULLET(object):
    def __init__(self):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552 
        self._num_steps = 0
        self._current_steps = 0
        self._timestep = 0.01
        self._experiment = None
        self._pybullet = 1
        self._simulator = "PyBullet"
        
        #Setup PyBullet
        p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.8)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,enable=0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,enable=0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,enable=0)
        p.setPhysicsEngineParameter(fixedTimeStep = self._timestep)
        
        #Setup Floor
        self._planeId = p.loadURDF("plane.urdf")
                
        #Robot Arm
        self._kinovaStartPos = [0,0,0.055]
        self._kinova = p.loadURDF("kinova_description/urdf/m1n6s300.urdf",self._kinovaStartPos,useFixedBase=1)
        #Cube
        self._cubedim = [0.0375,0.0375,0.0375]
        self._cube_mass = 0.0884
        self._visualShapeId = -1
        self._cubeStartPos = [0.5,0,0.0375]
        self._cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        p.createMultiBody(0,0)
        self._colCubeId = p.createCollisionShape(p.GEOM_BOX,halfExtents=self._cubedim)
        self._cubeUid = p.createMultiBody(self._cube_mass,self._colCubeId,self._visualShapeId,self._cubeStartPos,self._cubeStartOrientation)
        
        #Sim Switching
        self._next_sim = None
        self._switch_sim = False
               
    def set_experiment(self,experiment):
        self._experiment = experiment    
    
    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep)

    def set_current_steps(self,time):
        self._current_steps = time/self._timestep
    
    def run_pybullet(self,time,save):
        
        save.restore_pybullet_states(self._kinova,self._cubeUid)
        save = SAVE_STATE()
        self.set_num_steps()
        self.set_current_steps(time)
        
        for simStep in range(round(self._current_steps), self._num_steps):
            
            self._pid = set_positions(self._timestep, self._pid,self._experiment,self._simulator,simStep)
            
            if simStep % 5 == 0:
                for jointNum in range(8):
                    self._theta[jointNum] = p.getJointState(self._kinova, jointNum)[0]
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    p.setJointMotorControl2(bodyIndex=self._kinova,jointIndex=jointNum,controlMode=p.VELOCITY_CONTROL,targetVelocity=self._linearVelocity[jointNum],force=2000)
                self._positions.append([p.getLinkState(self._kinova,6,1)[0][0],p.getLinkState(self._kinova,6,1)[0][1],p.getLinkState(self._kinova,6,1)[0][2],p.getLinkState(self._kinova,6,1)[1][0],p.getLinkState(self._kinova,6,1)[1][1],p.getLinkState(self._kinova,6,1)[1][2],p.getLinkState(self._kinova,6,1)[1][3],p.getBasePositionAndOrientation(self._cubeUid)[0][0],p.getBasePositionAndOrientation(self._cubeUid)[0][1],p.getBasePositionAndOrientation(self._cubeUid)[0][2],p.getBasePositionAndOrientation(self._cubeUid)[1][0],p.getBasePositionAndOrientation(self._cubeUid)[1][1],p.getBasePositionAndOrientation(self._cubeUid)[1][2],p.getBasePositionAndOrientation(self._cubeUid)[1][3]])
    
            p.stepSimulation()
            sleep(0.01)
            
            self._next_sim,self._switch_sim = simulator_selector([],self._pybullet)
    
            if self._switch_sim or simStep == self._num_steps:
                save.save_pybullet_states(self._kinova,self._cubeUid)
                p.disconnect()
                return simStep*self._timestep, self._positions, self._next_sim, save
