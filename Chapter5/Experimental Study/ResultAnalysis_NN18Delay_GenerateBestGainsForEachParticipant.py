#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 11:56:12 2019

Result Analysis of the Single Pendulum Optimization.

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import os 

St = 0
Ed = 1

num_nodes = 5001
interval = 0.02
duration = (num_nodes-1)*interval

start_nodes = 8000


for subj in range(3, 9):
    for trial in range(1, 3):

        num_states = 4
        num_cons = 4
        Scaling = 50.0
        integration_method = 'midpoint'
        max_delay_time = 0.12
        half_state = int(num_states/2)
        
        num_par = 2 + num_states*2 + 2 + 2 
        
        store_path = 'NN18Delay/Analysis_Paper/Subj0'+str(subj)+'Pert'+str(trial)+'NN18Delay/'
        if not os.path.exists(store_path):
            os.makedirs(store_path)
            
        data_path = 'NN18Delay/Results/Subj0'+str(subj)+'/Pert'+str(trial)+'_GFL_NN18_delay60/'
        
        Accel_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/SwayL000'+str(trial+1)+'.txt')
        X_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/JointsGFL000'+str(trial+1)+'.txt')
        Para_string = ('Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt') 
        
        accel_meas = -np.loadtxt(Accel_String1, usecols=(3))[start_nodes:start_nodes+num_nodes]
        x_meas = np.loadtxt(X_String1, usecols=(1, 2, 3, 4,))[start_nodes:start_nodes+num_nodes, :]*3.1416/180.0
        parameter = np.loadtxt(Para_string)
        
        itheta_a = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int)
        itheta_h = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int) + 1
        
        k = 0
            
        sta = np.loadtxt(data_path + 'RMS_Sta_'+str(trial-1)+str(k)+'.txt')[1]
        Result = np.loadtxt(data_path + 'TrajectoryResult_'+str(trial-1)+str(k)+'.txt')
        
        if sta == 0 or sta == 1:

            fig2 = plt.figure(figsize=(12, 8))
            ax1 = fig2.add_subplot(211)
            plt.ylabel('Ankle angle (rad)')
            ax2 = fig2.add_subplot(212)
            plt.ylabel('Hip angle (rad)')
            plt.xlabel('Nodes')
            
            ax1.plot(x_meas[:, 0], '.', label='Simulated Motion')
            ax1.plot(Result[:, 0], label='Identified Motion')
            
            ax2.plot(x_meas[:, 1], '.', label='Simulated Motion')
            ax2.plot(Result[:, 1], label='Identified Motion')
            
            plt.legend()
            fig2.savefig(store_path + 'Motion_Comparison.png')
            
            rms = np.sqrt(sum((x_meas[:, 0] - Result[:, 0])**2 + (x_meas[:, 1] - Result[:, 1])**2)/num_nodes)
            
            _, _, r_v1, _, _ = stats.linregress(x_meas[:, 0], Result[:, 0])
            _, _, r_v2, _, _ = stats.linregress(x_meas[:, 1], Result[:, 1])
            
        else:
            rms = np.nan
            r_v1 = np.nan
            r_v2 = np.nan
                
        with open(store_path + 'Best_RMS_R.txt', 'w') as outfile:
            StringP = ''
            StringP += str(rms)
            StringP += '\n'
            StringP += str(r_v1**2)
            StringP += '\n'
            StringP += str(r_v2**2)
            StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path + 'Best_Traj.txt','w') as Outfile:
            StringP = ""
            for m in range(0,num_nodes):
                for n in range(0,num_states):
                    StringP += str(Result[m, n])
                    StringP += " "
                StringP += "\n"
            Outfile.write(StringP)