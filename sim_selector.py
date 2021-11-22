# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 14:09:31 2018

@author: jack
"""
import random

def simulator_selector(data,current_sim):
    #For the time being we are just going to randomly select simulators 4% of the time
    change_sim = random.randint(0,25)
    to_simulator = current_sim
    if change_sim == 0:
        switch_sim = True
        while to_simulator == current_sim:
            to_simulator = random.randint(0,1)
    else:
        to_simulator = current_sim
        switch_sim = False

    return to_simulator, switch_sim