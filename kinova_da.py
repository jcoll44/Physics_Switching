import pydart2 as pydart
from PID import *
from Rotations import *
from save_state import *
from sim_selector import *
import numpy as np
import csv
import math
import sys

class INFO(object):
    def __init__(self):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552
        self._current_steps = 0
        self._timestep = 0.0001
        self._num_steps = 0
        self._experiment = None
        self._dart = 5
        self._simulator = "Dart"
        
        #Sim Switching
        self._next_sim = None
        self._switch_sim = False


class DART(object):

    def __init__(self): 
        self.info = INFO()
        pydart.init()
        
    def set_experiment(self,experiment):
        self.info._experiment = experiment
    
    def set_num_steps(self):
        self.info._num_steps = simSteps(self.info._experiment,self.info._timestep)
        
    def set_current_steps(self,time):
        self.info._current_steps = time/self.info._timestep
    
    def run_dart(self,time,save):
        
        self.world = MyWorld(self.info)
        
        save.restore_dart_states(self.world)
        self.info.save = SAVE_STATE()
        self.set_num_steps()
        self.set_current_steps(time)
        
        
        from pydart2.gui.pyqt5.window import PyQt5Window
        self.info.win = PyQt5Window(self.world, None)
        self.info.win.scene.set_camera(1)
        self.info.win.playAction.setChecked(True)
        self.info.win.run()
        
        return self.info.time, self.info._positions, self.info._next_sim, self.info.save

   
class MyWorld(pydart.World):

    def __init__(self, info):
        self.info = info
        pydart.World.__init__(self, self.info._timestep,"./kinova_description/urdf/softBodies.skel")
        self.set_gravity([0,0,-9.81])
#        self.skeletons[1].set_root_joint_to_trans_and_euler()
           
        self.robot = self.add_skeleton("./kinova_description/urdf/m1n6s300.urdf")
        self.positions = self.robot.positions()
        self.positions["rootJoint_pos_z"] = 0.055
        self.robot.set_positions(self.positions)
        for i in range(6):
            self.robot.joints[i].set_actuator_type(pydart.joint.Joint.VELOCITY)
        for i in range(6,10):
            self.robot.joints[i].set_actuator_type(pydart.joint.Joint.LOCKED)
        #Initialise Controller
        self.controller = Controller(self.robot, self.info)

                
    def step(self, ):
        if self.nframes+round(self.info._current_steps) <= self.info._num_steps:
            
            self.info._pid = set_positions(self.info._timestep, self.info._pid,self.info._experiment,self.info._simulator,self.nframes+round(self.info._current_steps))

            if self.nframes % 500 == 0:
                self.controller.compute()
                self.check()
                
                if self.info._switch_sim or self.nframes+round(self.info._current_steps) == self.info._num_steps:
                    self.info.save.save_dart_states(self)
                    self.info.win.close()
                    self.info.time = self.nframes+round(self.info._current_steps)*self.info._timestep
                    return

            super(MyWorld, self).step()
            
            
            
        
    def check(self, ):
        self.info._next_sim,self.info._switch_sim = simulator_selector([],self.info._dart)
            

        
class Controller(object,):

    def __init__(self, robot, info):
        self.info = info
        self.robot = robot
        self.speeds = np.zeros(15)

    def compute(self, ):
        for jointNum in range(6):
            self.info._theta[jointNum] = self.robot.positions()[jointNum+6]
            self.info._linearVelocity[jointNum] = self.info._pid[jointNum].get_velocity(math.degrees(self.info._theta[jointNum]))/self.info._convertdeg2rad
            self.speeds[jointNum+6:] = self.info._linearVelocity[jointNum]
        self.robot.set_velocities(self.speeds)
    