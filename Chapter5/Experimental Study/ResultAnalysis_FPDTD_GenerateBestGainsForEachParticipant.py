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
Ed = 10

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
        
        store_path = 'FPDTD/Analysis_Paper/Subj0'+str(subj)+'Pert'+str(trial)+'GoodWorth5/'
        if not os.path.exists(store_path):
            os.makedirs(store_path)
            
        data_path = 'FPDTD/Results/Subj0'+str(subj)+'Pert'+str(trial)+'GoodWorth5/'
        
        Accel_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/SwayL000'+str(trial+1)+'.txt')
        X_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/JointsGFL000'+str(trial+1)+'.txt')
        Para_string = ('Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt') 
        
        accel_meas = -np.loadtxt(Accel_String1, usecols=(3))[start_nodes:start_nodes+num_nodes]
        x_meas = np.loadtxt(X_String1, usecols=(1, 2, 3, 4,))[start_nodes:start_nodes+num_nodes, :]*3.1416/180.0
        parameter = np.loadtxt(Para_string)
        
        itheta_a = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int)
        itheta_h = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int) + 1
        
        opt_sta = np.zeros((int(Ed - St), 2))
        opt_con = np.zeros((int(Ed - St), num_par))
        
        for k in range(St, Ed):
            opt_sta[k, :] = np.loadtxt(data_path + 'RMS_Sta_'+str(k)+'.txt')[:2]
            opt_con[k, :] = np.loadtxt(data_path + 'Controller_'+str(k)+'.txt')
            
        sta_ind = np.where(opt_sta[:, 1]==0)[0]
        rms_ind = np.argsort(opt_sta[sta_ind, 0])
        con_sort = opt_con[sta_ind[rms_ind], :]
        
        fig3 = plt.figure()
        index = np.linspace(1, len(sta_ind), len(sta_ind), endpoint=True)
        plt.bar(index, opt_sta[sta_ind[rms_ind], 0])
        
        
        if not len(sta_ind) == 0:
            
            aver_numb = len(np.where(((opt_sta[sta_ind[rms_ind], 0] - opt_sta[sta_ind[rms_ind][0], 0])/opt_sta[sta_ind[rms_ind][0], 0]) < 0.01)[0])
            mean_con_close = np.mean(con_sort[:aver_numb, :], axis=0)
            std_con_close = np.std(con_sort[:aver_numb, :], axis=0)
            best_con_close = con_sort[0, :]
            
            mean_con_close[:2 + num_states*2] = mean_con_close[:2 + num_states*2]*Scaling
            std_con_close[:2 + num_states*2] = std_con_close[:2 + num_states*2]*Scaling
            best_con_close[:2 + num_states*2] = best_con_close[:2 + num_states*2]*Scaling
            
        #    # change time delay to ms
        #    mean_con_close[2 + num_states*2:2 + num_states*2 + 2] = mean_con_close[2 + num_states*2:2 + num_states*2 + 2]
        #    std_con_close[2 + num_states*2:2 + num_states*2 + 2] = std_con_close[2 + num_states*2:2 + num_states*2 + 2]
        #    
        #    # change reference angle to degree
        #    mean_con_close[2 + num_states*2 + 2:2 + num_states*2 + 4] = mean_con_close[2 + num_states*2 + 2:2 + num_states*2 + 4]
        #    std_con_close[2 + num_states*2 + 2:2 + num_states*2 + 4] = std_con_close[2 + num_states*2 + 2:2 + num_states*2 + 4]
            
            width = 0.35
            ind1 = np.arange(4)    # the x locations for the groups
            ind2 = np.arange(2)
            
            fig1 = plt.figure(figsize=(12, 8))
            ax1 = fig1.add_subplot(3,2,1)
            plt.ylabel('Passive Gains for Ankle and Hip')
            plt.xticks(ind1, ('Kp_a', 'Kp_h'))
            ax2 = fig1.add_subplot(3,2,2)
            plt.ylabel('Passive Gains for Hip')
            plt.xticks(ind1, ('Kp_a', 'Kp_h', 'Kd_a', 'Kd_h'))
            ax3 = fig1.add_subplot(3,2,3)
            plt.ylabel('Active Gains for Ankle')
            plt.xticks(ind1, ('Kp_a', 'Kp_h', 'Kd_a', 'Kd_h'))
            ax4 = fig1.add_subplot(3,2,4)
            plt.ylabel('Active Gains for Hip')
            plt.xticks(ind1, ('Kp_a', 'Kp_h', 'Kd_a', 'Kd_h'))
            ax5 = fig1.add_subplot(3,2,5)
            plt.ylabel('Delay Time /ms')
            plt.xticks(ind2, ('Td_a', 'Td_h'))
            ax6 = fig1.add_subplot(3,2,6)
            plt.ylabel('Reference Angle /deg')
            plt.xticks(ind2, ('Re_a', 'Re_h'))
            
            ax1.bar(ind2, mean_con_close[:2], width, yerr=std_con_close[:2], error_kw=dict(elinewidth=4, ecolor='red'))
            #ax2.bar(ind1, mean_con_close[4:8], width, yerr=std_con_close[4:8], error_kw=dict(elinewidth=4, ecolor='red'))
            ax3.bar(ind1, mean_con_close[2:6], width, yerr=std_con_close[2:6], error_kw=dict(elinewidth=4, ecolor='red'))
            ax4.bar(ind1, mean_con_close[6:10], width, yerr=std_con_close[6:10], error_kw=dict(elinewidth=4, ecolor='red'))
            
            ax5.bar(ind2, mean_con_close[10:12], width, yerr=std_con_close[10:12], error_kw=dict(elinewidth=4, ecolor='red'))
            ax6.bar(ind2, mean_con_close[12:14], width, yerr=std_con_close[12:14], error_kw=dict(elinewidth=4, ecolor='red'))
            
            fig1.savefig(store_path + 'Identified_Control_Gains.png')
            
            with open(store_path + 'averaged_gains.txt', 'w') as outfile:
                StringP = ''
                for g in range(num_par):
                    StringP += str(mean_con_close[g])
                    StringP += ' '
                StringP += '\n'
                outfile.write(StringP)
                
            fig1.savefig(store_path + 'Identified_Control_Gains.png')
            
            with open(store_path + 'STD_gains.txt', 'w') as outfile:
                StringP = ''
                for g in range(num_par):
                    StringP += str(std_con_close[g])
                    StringP += ' '
                StringP += '\n'
                outfile.write(StringP)
                
            with open(store_path + 'Best_gains.txt', 'w') as outfile:
                StringP = ''
                for g in range(num_par):
                    StringP += str(best_con_close[g])
                    StringP += ' '
                StringP += '\n'
                outfile.write(StringP)
           
            k = sta_ind[rms_ind[0]]
            
        else:
            k = 0
            
        Result = np.loadtxt(data_path + 'TrajectoryResult_'+str(k)+'.txt')
        Residule = np.loadtxt(data_path + 'ConstraintsResidule_'+str(k)+'.txt')
        
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
