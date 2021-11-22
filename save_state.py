# -*- coding: utf-8 -*-
"""
Created on Tue Oct  9 15:54:29 2018

@author: jack
"""
#Mujoco
from mujoco_py import *
#PyBullet
import pybullet as p
import pybullet_data
#V-Rep
import vrep
import sys
#Dart
import pydart2 as pydart

import math as math
import numpy as np


class SAVE_STATE(object):
    def __init__(self):
        
        # State of the robot arm
        self._arm_position = [] #In this case the rotations of each joint
        self._arm_velocities = []
        self._arm_forces = []
        
        # State of the cube
        self._cube_position= [] # x,y,z global + w,x,y,z (quaternion)
        self._cube_velocities = [] # [x,y,z,wx,wy,wz] 
        self._cube_forces = []
        
    def printLists(self):
        print(self._arm_position)
        print(self._arm_velocities)
        print(self._cube_position)
        print(self._cube_velocities)
        
    def save_mujoco_states(self,sim):
#        self._positions.append([self._sim.data.get_body_xpos('m1n6s300_link_6')[0],self._sim.data.get_body_xpos('m1n6s300_link_6')[1],self._sim.data.get_body_xpos('m1n6s300_link_6')[2],self._sim.data.get_body_xquat('m1n6s300_link_6')[0],self._sim.data.get_body_xquat('m1n6s300_link_6')[1],self._sim.data.get_body_xquat('m1n6s300_link_6')[2],self._sim.data.get_body_xquat('m1n6s300_link_6')[3],self._sim.data.get_body_xpos('cube')[0],self._sim.data.get_body_xpos('cube')[1],self._sim.data.get_body_xpos('cube')[2],self._sim.data.get_body_xquat('cube')[0],self._sim.data.get_body_xquat('cube')[1],self._sim.data.get_body_xquat('cube')[2],self._sim.data.get_body_xquat('cube')[3]])
        for ID in range(9):        
            self._arm_position.append(sim.data.get_joint_qpos(sim.model.joint_id2name(ID)))
            self._arm_velocities.append(sim.data.get_joint_qvel(sim.model.joint_id2name(ID)))
            
        self._cube_position = sim.data.get_joint_qpos('cube')
        self._cube_velocities = sim.data.get_joint_qvel('cube')
        
    def save_pybullet_states(self, kinova, cubeUid):
        for ID in range(2,11):
            self._arm_position.append(p.getJointState(kinova,ID)[0])
            self._arm_velocities.append(p.getJointState(kinova,ID)[1])
        
        self._cube_position.extend(p.getBasePositionAndOrientation(cubeUid)[0])
        self._cube_position.extend(p.getBasePositionAndOrientation(cubeUid)[1])
        self._cube_position[3:] = self._cube_position[4:] + [self._cube_position[3]]
        
        self._cube_velocities.extend(p.getBaseVelocity(cubeUid)[0])
        self._cube_velocities.extend(p.getBaseVelocity(cubeUid)[1])
        

    
    def save_vrep_states(self,clientID,arm_joints,cube):
        for ID in range(9):
            self._arm_position.append(vrep.simxGetJointPosition(clientID,arm_joints[ID],vrep.simx_opmode_blocking)[1])
            self._arm_velocities.append(vrep.simxGetObjectFloatParameter(clientID,arm_joints[ID],2012,vrep.simx_opmode_blocking)[1])
            
        self._cube_position.extend(vrep.simxGetObjectPosition(clientID,cube,-1,vrep.simx_opmode_blocking)[1])
        self._cube_position.extend(vrep.simxGetObjectQuaternion(clientID,cube,-1,vrep.simx_opmode_blocking)[1])
        self._cube_velocities.extend(vrep.simxGetObjectVelocity(clientID,cube,vrep.simx_opmode_blocking)[1])
        self._cube_velocities.extend(vrep.simxGetObjectVelocity(clientID,cube,vrep.simx_opmode_blocking)[2])
        
    def save_dart_states(self, world):
        for ID in range(9):
            self._arm_position.append(world.robot.positions()[ID+6])
            self._arm_velocities.append(world.robot.velocities()[ID+6])
        
        cube_transform = world.skeletons[1].body('link 1').world_transform()
        self._cube_position.extend(cube_transform[3][0])
        self._cube_position.extend(cube_transform[3][1])
        self._cube_position.extend(cube_transform[3][2])
        w = sqrt(1 + cube_transform[0][0] + cube_transform[1][1] + cube_transform[2][2])/2
        x = (cube_transform[2][1]-cube_transform[1][2])/(4*w)
        y = (cube_transform[0][2]-cube_transform[2][0])/(4*w)
        z = (cube_transform[1][0]-cube_transform[0][1])/(4*w)
        self._cube_position.extend([w,x,y,z])
        
        self._cube_velocities.extend(world.skeletons[1].body('link 1').com_spatial_velocity())
        

    
    def restore_mujoco_states(self,sim):
        for ID in range(9):
            sim.data.set_joint_qpos(sim.model.joint_id2name(ID),self._arm_position[ID])
            sim.data.set_joint_qvel(sim.model.joint_id2name(ID),self._arm_velocities[ID])
            
        sim.data.set_joint_qpos('cube', self._cube_position)
        sim.data.set_joint_qvel('cube', self._cube_velocities)
        sim.forward()
        
    def restore_pybullet_states(self,kinova,cubeUid):
        for ID in range(2,11):
            p.resetJointState(kinova,ID,targetValue = self._arm_position[ID-2], targetVelocity = self._arm_velocities[ID-2])

        self._cube_position[3:] = [self._cube_position[6]] + self._cube_position[3:6]
        p.resetBasePositionAndOrientation(cubeUid, posObj = self._cube_position[0:3], ornObj = self._cube_position[3:] )
        p.resetBaseVelocity(cubeUid,linearVelocity = self._cube_velocities[0:3], angularVelocity = self._cube_velocities[3:])
        
    def restore_vrep_states(self,clientID,arm_joints,cube):
        if self._arm_position:
            for ID in range(9):
                vrep.simxSetJointPosition(clientID,arm_joints[ID],self._arm_position[ID],vrep.simx_opmode_blocking)
                vrep.simxSetJointTargetVelocity(clientID,arm_joints[ID],self._arm_velocities[ID],vrep.simx_opmode_blocking) 
            
            vrep.simxSetObjectPosition(clientID,cube,-1,self._cube_position[0:3],vrep.simx_opmode_blocking)
            vrep.simxSetObjectQuaternion(clientID,cube,-1,self._cube_position[3:7],vrep.simx_opmode_blocking)
    
    def restore_dart_states(self, world):
        arm_positions = np.zeros(15)
        arm_positions[6:15] = self._arm_position
        world.robot.set_positions(arm_positions)
        arm_velocities = np.zeros(15)
        arm_velocities[6:15] = self._arm_velocities
        world.robot.set_velocities(arm_velocities)
        
        world.positions = world.skeletons[1].positions()
        euler = euler_from_quaternion(self._cube_position[3:7])
        world.positions[:] = euler+self._cube_position[0:3]
        world.skeletons[1].set_positions(world.positions)
        
        world.velocities = world.skeletons[1].velocities()
        world.velocities = self._cube_velocities
        world.skeletons[1].set_velocities(self._cube_velocities)
            
        
def euler_from_quaternion(quat):
    #Roll
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    #Pitch
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2,sinp)
    else:
        pitch = math.asin(sinp)
    
    #Yaw
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll,pitch,yaw]