#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 19 15:08:24 2019

This is forward simulation of a single pendulum under perturbation with PD feedback
control with delay.

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import root
from numpy import sin, cos, pi
import os
from scipy.interpolate import interp1d
import copy

num_nodes = 10001
interval = 0.01
duration = (num_nodes-1)*interval

start_nodes = 8000

subj = 4
trial = 1

num_states = 4
num_cons = 4
scaling = 1.0
integration_method = 'midpoint'
max_delay_time = 0.16
half_state = int(num_states/2)

num_par = 2 + num_states*2 + 2 + 2

Dictionary = 'ForwardSimulation_GoodWorth/'
if not os.path.exists(Dictionary):
    os.makedirs(Dictionary)
    
Accel_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/SwayL000'+str(trial+1)+'.txt')
X_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/JointsGFL000'+str(trial+1)+'.txt')
Para_string = ('Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt')

accel_exp = np.loadtxt(Accel_String1)
accel_meas_50Hz = -accel_exp[start_nodes:start_nodes+int((num_nodes-1)/2), 3]
time_50Hz = accel_exp[start_nodes:start_nodes+int((num_nodes-1)/2), 0] - accel_exp[start_nodes, 0]
time_100Hz = np.linspace(0, duration, num_nodes, endpoint=True)
accel_meas_100Hz = np.interp(time_100Hz, time_50Hz, accel_meas_50Hz)
parameter = np.loadtxt(Para_string)

#fig = plt.figure()
#plt.plot(time_50Hz, accel_meas_50Hz)
#plt.plot(time_100Hz, accel_meas_100Hz, '.')

for kk in range(50):

    pink_noise1 = np.loadtxt('ForwardSimulation_GoodWorth/pink_noise1_10000_'+str(kk)+'.txt')
    pink_noise2 = np.loadtxt('ForwardSimulation_GoodWorth/pink_noise2_10000_'+str(kk)+'.txt')  
    
    Con_GoodWorth = np.array([152.76, 19.09, 891.07, 286.42, 151.48, 19.09, 251.41,
                              175.03, 50.70, 28.00, 0.09, 0.08, 0, 0])
        
    states = np.zeros((4, num_nodes))
                
    tor = np.zeros((2, num_nodes))
    noise = np.zeros((2, num_nodes-1))
    noise[0, :] = pink_noise1
    noise[1, :] = pink_noise2
    

    
    def dyn(Y, tor, a):
        # this is the dynamics equation of the single pendulum
        
        l_L = parameter[0]
        I_T = parameter[6]/scaling
        g = 9.81
        d_T = parameter[2]
        d_L = parameter[1]
        I_L = parameter[5]/scaling
        m_L = parameter[3]/scaling
        m_T = parameter[4]/scaling
        
        theta_a = Y[0]
        theta_h = Y[1]
        omega_a = Y[2]
        omega_h = Y[3]
        
        M = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, (I_L + I_T + d_L**2*m_L + m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)), (I_T + d_T*m_T*(d_T + l_L*cos(theta_h)))],
          [0, 0, (I_T + d_T*m_T*(d_T + l_L*cos(theta_h))), (I_T + d_T**2*m_T)]])
        
        ForceVec = np.matrix([[omega_a], [omega_h], [(a*d_L*m_L*cos(theta_a) + a*d_T*m_T*cos(theta_a + theta_h) 
                + a*l_L*m_T*cos(theta_a) + d_L*g*m_L*sin(theta_a) + d_T*g*m_T*sin(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*sin(theta_h) 
                + g*l_L*m_T*sin(theta_a) + tor[0])],
        
                    [(a*d_T*m_T*cos(theta_a + theta_h) + d_T*g*m_T*sin(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*sin(theta_h) + tor[1])]])
        
        
        M_rev = M.I
        
        DY =np.dot(M_rev, ForceVec)
        DY = np.asarray(DY).reshape(-1)
        
        return DY
    
    
    for k in range(1, num_nodes-1):
        
        def obj(x):
            
            states[:, k] = x
            
            Kp_passive = Con_GoodWorth[:2]
            Con_a = Con_GoodWorth[2:6]
            Con_h = Con_GoodWorth[6:10]
            
            delay_node1 = Con_GoodWorth[-4]/interval
            delay_node2 = Con_GoodWorth[-3]/interval
            
            Ref = np.array([Con_GoodWorth[-2], Con_GoodWorth[-1], 0, 0])
                
            if k <= int(delay_node1):
                delay_states1 = (states[:, 0]+states[:, 1])/2
                
                delay_states1[0] = delay_states1[0] + (noise[0, 0] + noise[0, 1])/2
                delay_states1[1] = delay_states1[1] + (noise[1, 0] + noise[1, 1])/2
                
            elif k == int(delay_node1) + 1:
                
                xp = copy.deepcopy(states[:, k-int(delay_node1)-1])
                xn = (states[:, k-int(delay_node1)-1] + states[:, k-int(delay_node1)])/2
                
                xp[0] = xp[0] + noise[0, k-int(delay_node1)-1]
                xp[1] = xp[1] + noise[1, k-int(delay_node1)-1]
                
                xn[0] = xn[0] + (noise[0, k-int(delay_node1)-1] + noise[0, k-int(delay_node1)])/2
                xn[1] = xn[1] + (noise[1, k-int(delay_node1)-1] + noise[1, k-int(delay_node1)])/2
                
                delay_states1 = ((1 - delay_node1 + int(delay_node1))*xn + (delay_node1 - int(delay_node1))*xp)
                
            else:
                
                xp = (states[:, k-int(delay_node1)-1] + states[:, k-int(delay_node1)-2])/2
                xn = (states[:, k-int(delay_node1)-1] + states[:, k-int(delay_node1)])/2
                
                xp[0] = xp[0] + (noise[0, k-int(delay_node1)-1] + noise[0, k-int(delay_node1)-2])/2
                xp[1] = xp[1] + (noise[1, k-int(delay_node1)-1] + noise[1, k-int(delay_node1)-2])/2                
                
                xn[0] = xn[0] + (noise[0, k-int(delay_node1)-1] + noise[0, k-int(delay_node1)])/2
                xn[1] = xn[1] + (noise[1, k-int(delay_node1)-1] + noise[1, k-int(delay_node1)])/2
                
                delay_states1 = ((1 - delay_node1 + int(delay_node1))*xn + (delay_node1 - int(delay_node1))*xp)
                
            if k <= int(delay_node2):
                delay_states2 = (states[:, 0]+states[:, 1])/2
                
                delay_states2[0] = delay_states2[0] + (noise[0, 0] + noise[0, 1])/2
                delay_states2[1] = delay_states2[1] + (noise[1, 0] + noise[1, 1])/2
                
            elif k == int(delay_node2) + 1:
                
                xp = copy.deepcopy(states[:, k-int(delay_node2)-1])
                xn = (states[:, k-int(delay_node2)-1] + states[:, k-int(delay_node2)])/2
                
                xp[0] = xp[0] + noise[0, k-int(delay_node2)-1]
                xp[1] = xp[1] + noise[1, k-int(delay_node2)-1]
                
                xn[0] = xn[0] + (noise[0, k-int(delay_node2)-1] + noise[0, k-int(delay_node2)])/2
                xn[1] = xn[1] + (noise[1, k-int(delay_node2)-1] + noise[1, k-int(delay_node2)])/2
                
                delay_states2 = ((1 - delay_node2 + int(delay_node2))*xn + (delay_node2 - int(delay_node2))*xp)
                
            else:
                
                xp = (states[:, k-int(delay_node2)-1] + states[:, k-int(delay_node2)-2])/2
                xn = (states[:, k-int(delay_node2)-1] + states[:, k-int(delay_node2)])/2
                
                xp[0] = xp[0] + (noise[0, k-int(delay_node2)-1] + noise[0, k-int(delay_node2)-2])/2
                xp[1] = xp[1] + (noise[1, k-int(delay_node2)-1] + noise[1, k-int(delay_node2)-2])/2                
                
                xn[0] = xn[0] + (noise[0, k-int(delay_node2)-1] + noise[0, k-int(delay_node2)])/2
                xn[1] = xn[1] + (noise[1, k-int(delay_node2)-1] + noise[1, k-int(delay_node2)])/2
                
                delay_states2 = ((1 - delay_node2 + int(delay_node2))*xn + (delay_node2 - int(delay_node2))*xp)
                
            # add noise to delayed nodes
#            delay_states1[0] = delay_states1[0] + noise[0, k]
#            delay_states1[1] = delay_states1[1] + noise[1, k]
#            
#            delay_states2[0] = delay_states2[0] + noise[0, k]
#            delay_states2[1] = delay_states2[1] + noise[1, k]
            
            x_with_noise = np.zeros(4)
            
            x_with_noise[:2] = (states[:2, k-1]+states[:2, k])/2 + (noise[:, k-1] + noise[:, k])/2
            x_with_noise[2:] = (states[2:, k-1]+states[2:, k])/2 
            
            tor[0, k] = (Kp_passive[0]*(Ref[0] - x_with_noise[0]) +
                            np.dot(Con_a, Ref-delay_states1))
        
            tor[1, k] = (Kp_passive[1]*(Ref[1] - x_with_noise[1]) +
                            np.dot(Con_h, Ref-delay_states2))
            
            theta_dd = dyn(x_with_noise, tor[:, k], (accel_meas_100Hz[k-1] + accel_meas_100Hz[k])/2)
            f = np.zeros(num_states)
            f[:2] = theta_dd[:2] - (states[:2, k] - states[:2, k-1])/interval
            f[2:] = theta_dd[2:] - (states[2:, k] - states[2:, k-1])/interval
            
            return f
        
        res = root(obj, states[:, k-1])
        
        states[:, k] = res.x

    save_name = Dictionary + 'DoublePendulum_N'+str(num_nodes)+'Noise_'+str(kk)+'.txt'
    
    with open(save_name, 'w') as outfile:
        StringP = ''
        for i in range(num_nodes):
            for j in range(num_states):
                StringP += str(states[j, i])
                StringP += ' '
            StringP += '\n'
        outfile.write(StringP)
            
            
        
        
    

    