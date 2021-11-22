# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 13:55:07 2018

@author: jack
"""
from PID import *
import subprocess
import time
import csv
import math

import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import matplotlib.pyplot
#from mpl_toolkits.mplot3d import *

def simSteps(experiment,timestep):      
    if experiment == "Test":
        seconds = 6
    elif experiment == "Single":
        seconds = 6
    elif experiment == "Double":
        seconds = 20
    elif experiment == "Cube":
        seconds = 20
        
    return int(seconds/timestep)
        
def set_positions(timestep, pid, experiment,simulator, simStep):
    if experiment == "Test":
        if timestep*simStep < 3:
            pid = set_end_point([0.145,-0.145, 0.1],pid,simulator)
        elif timestep*simStep >= 3:
            pid = set_end_point([0,-0.145, 0.5],pid,simulator)
            
    elif experiment == "Single":
        if simStep == 0:
            pid[1].set_target_theta(-100)
        else:
            return pid
        if simulator == "PyBullet":
            pid = pid[-2:] + pid[0:-2]
            
    elif experiment == "Double":
        if simStep == 0:
            pid[1].set_target_theta(-90)
            pid[4].set_target_theta(-90)
        elif simStep % (num_steps*0.5) == 0:
            pid[1].set_target_theta(-90)
            pid[4].set_target_theta(90)
        elif simStep % (num_steps*0.25) == 0:
            pid[1].set_target_theta(0)
            pid[4].set_target_theta(0)
        else:
            return pid
        if simulator == "PyBullet":
            pid = pid[-2:] + pid[0:-2]
            
    elif experiment == "Cube":
        if simStep == 0:
            pid[1].set_target_theta(30)
            pid[2].set_target_theta(120)
            pid[3].set_target_theta(90)
        elif  num_steps*0.4 == simStep:
            pid[1].set_target_theta(-60)
            pid[2].set_target_theta(65)
            pid[3].set_target_theta(90)
        elif num_steps*0.7 == simStep:
            pid[1].set_target_theta(-67)
            pid[2].set_target_theta(30)
            pid[3].set_target_theta(90)
        else:
            return pid
        if simulator == "PyBullet":
            pid = pid[-2:] + pid[0:-2]
            
    return pid
    
def open_vrep():
    proc = subprocess.Popen(["cd /home/jack/repos/V-REP_PRO_EDU_V3_5_0_Linux && ./vrep.sh"], shell=True)
    time.sleep(8)
    proc.terminate()

def saveStats(experiment, iteration, physics_engine, simulator, positions):
    #Save simulation Data     
    fileString = 'Results/%s_%s%s_%d.csv'%(experiment,simulator,physics_engine,iteration)
    with open(fileString, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
        for xyz in positions:
            writer.writerow(xyz)
            
def set_end_point(target_vector,pid, simulator):
    target_thetas = inverseKinematics(target_vector)
    
    for i in range(6):
        pid[i].set_target_theta(target_thetas[i])
    
    if simulator == "PyBullet":
        pid = pid[-2:] + pid[0:-2] 
    
    return pid
        


def inverseKinematics(target_vector):
#    my_chain = ikpy.chain.Chain.from_urdf_file("/home/jack/Documents/PhD/Code/kinova_description/urdf/m1n6s300.urdf", base_elements=["root", "world", "m1n6s300_link_base", "m1n6s300_link_finger_1", "m1n6s300_link_finger_tip_1","m1n6s300_link_finger_2", "m1n6s300_link_finger_tip_2", "m1n6s300_link_finger_3","m1n6s300_link_finger_tip_3"], active_links_mask=[False,False, False, True, True, True, True, True, True, True, False, False, False, False, False, False ])
    my_chain = Chain(name='kinova', links=[
    OriginLink(),
        URDFLink(
        name="m1n6s300_joint_1",
            translation_vector=[0, 0, 0.15675],
            orientation=[0, 3.14159265359, 0],
            rotation=[0, 0, 1],
        ),
        URDFLink(
        name="m1n6s300_joint_2",
            translation_vector=[0, 0.0016, -0.11875],
            orientation=[-1.57079632679, 3.14159265359, 3.14159265359],
            rotation=[0, 0, 1],
            bounds =[-2.35, 2.35], 
        ),
        URDFLink(
        name="m1n6s300_joint_3",
            translation_vector=[0, -0.290, 0],
            orientation=[0, 3.14159265359, 3.14159265359],
            rotation=[0, 0, 1],
            bounds =[-2.35, 2.35],
        ),
        URDFLink(
        name="m1n6s300_joint_4",
            translation_vector=[0, 0.1231, -0.0086],
            orientation=[-1.57079632679, 0, 3.14159265359],
            rotation=[0, 0, 1],
        ),
        URDFLink(
        name="m1n6s300_joint_5",
            translation_vector=[0, -0.03703, -0.06414],
            orientation=[1.0471975512, 0, 3.14159265359],
            rotation=[0, 0, 1],
        ),
        URDFLink(
        name="m1n6s300_joint_6",
            translation_vector=[0, -0.03703, -0.06414],
            orientation=[1.0471975512, 0, 3.14159265359],
            rotation=[0, 0, 1],
        ),
        URDFLink(
        name="m1n6s300_joint_end_effector",
            translation_vector=[0, 0, -0.1600],
            orientation=[3.14159265359, 0, 0],
            rotation=[0, 0, 1],
        )
        ])
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_vector
#    print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))
    
#    plotIK(my_chain,target_frame)    
    
    inverse_angles = my_chain.inverse_kinematics(target_frame)[1:7]    
    
    for i in range(6):
        if abs(math.degrees(inverse_angles[i])) > 360:
            inverse_angles[i] = math.degrees(inverse_angles[i]) % 360
        else:
            inverse_angles[i] = math.degrees(inverse_angles[i])
        if inverse_angles[i] > 180:
            inverse_angles[i] = 360-inverse_angles[i]
        elif inverse_angles[i] < -180:
            inverse_angles[i] = -360-inverse_angles[i]
            
#    print(inverse_angles)
    return inverse_angles
    
    
def plotIK(my_chain,target_frame):
    
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    my_chain.plot(my_chain.inverse_kinematics(target_frame), ax,show=True)
    matplotlib.pyplot.show()
    
        
#def set_target_thetas(num_steps, pid, experiment,simulator, simStep):


    
    