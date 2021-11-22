# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 12:01:49 2018

@author: jack
"""
from kinova_mj import *
from kinova_pb import *
from kinova_vr import *
from kinova_da import *
from Rotations import *
from save_state import *
         
experiment = "Test"      
repeats = 1
simulator = "Combined"
physics_engine = ""

positions = []
time = 0
next_sim = None
not_complete = True
            
if __name__ == '__main__':
    
#    open_vrep()
    
    #        save = SAVE_STATE()
#        simulate = DART()
#        simulate.set_experiment(experiment)
#        time, positions, next_sim, save = simulate.run_dart(time,save)    
#        print("It Worked")
    
    for iteration in range(repeats):
                
        save = SAVE_STATE()
        simulate = VREP()
        simulate.set_experiment(experiment)
        time, positions, next_sim, save = simulate.run_vortex(time,save)    
    
        while not_complete:
            save.printLists()
            print(next_sim)
            if next_sim == 0:
                simulate = MUJOCO()
                simulate.set_experiment(experiment)
                time, positions, next_sim, save = simulate.run_mujoco(time,save)
            elif next_sim == 5:
                simulate = PYBULLET()
                simulate.set_experiment(experiment)
                time, positions, next_sim, save = simulate.run_pybullet(time,save)
            elif next_sim == 2:
                simulate = VREP()
                simulate.set_experiment(experiment)
                time, positions, next_sim, save = simulate.run_bullet278(time,save)
            elif next_sim == 3:
                simulate = VREP()
                simulate.set_experiment(experiment)
                time, positions, next_sim, save = simulate.run_vortex(time,save)
            elif next_sim == 4:
                simulate = VREP()
                simulate.set_experiment(experiment)
                time, positions, next_sim, save = simulate.run_newton(time,save)
            elif next_sim == 1:
                simulate = DART()
                simulate.set_experiment(experiment)
                time, positions, next_sim, save = simulate.run_dart(time,save)
            
            if time == simSteps(experiment, 1):
                not_complete = False
                
        saveStats(experiment,iteration,physics_engine,simulator,positions)
        
        
#Old Stuff, can possibly remove       
#    for iteration in range(repeats):
#        simulate = MUJOCO()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.run_mujoco()
##    
#    for iteration in range(repeats):
#        simulate = PYBULLET()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.run_pybullet()
#        
#    
#    for iteration in range(repeats):
#        simulate = VREP()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.run_simulation()
        
#    for iteration in range(repeats):
#        simulate = VREP()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.Bullet283()
#    for iteration in range(repeats):
#        simulate = VREP()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.Bullet278()
#    for iteration in range(repeats):
#        simulate = VREP()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.ODE()
#    for iteration in range(repeats):
#        simulate = VREP()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.Vortex()
#    for iteration in range(repeats):
#        simulate = VREP()
#        simulate.set_current_iteration(iteration)
#        simulate.set_experiment(experiment)
#        simulate.Newton()