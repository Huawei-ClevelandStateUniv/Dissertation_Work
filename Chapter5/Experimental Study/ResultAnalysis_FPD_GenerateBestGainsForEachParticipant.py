# -*- coding: utf-8 -*-
"""
Created on Thu Apr 13 09:46:29 2017

Result analysis of RepIniCon Trails


@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from IPython.core.pylabtools import figsize
#
import os
from scipy import signal
from scipy import stats

for Subj in range(3, 9):
    for Trail in range(1, 3):
#        Subj = 3
#        Trail = 1
        
        RepTimes = 10
        Scaling = 100.0
        Show_Num = 10
        
        start_nodes = 8000
        end_nodes = 13001
        start_time = start_nodes/50.0
        end_time = end_nodes/50.0
        
        num_states = 4
        num_repeat = 3
        num_nodes = 5001
        
        num_par = num_states + num_states + 2
                
        StoreSign = 'True'    # change to 'True' if you want to store analysis result
        
        StorePath = 'FPD/Analysis_Paper/Subj0'+str(Subj)+'/Pert'+str(Trail)+'_GFL_FPD_Eps'+str(num_repeat)+'_Noise025/'
        if not os.path.exists(StorePath):
            os.makedirs(StorePath)
        
        
        folder = 'FPD/Result/Subj0'+str(Subj)+'/Pert'+str(Trail)+'_GFL_FPD_Eps'+str(num_repeat)+'_Noise025/'
        
        folderE = ('Data/2DoF Data Simplify/Subj0'+str(Subj)+'/')
        # Get RMS sort
        opt_sta = np.zeros((RepTimes, 2))
        opt_con = np.zeros((RepTimes, num_par))
        
        for i in range(0, RepTimes):
            
            RMSSta0 = np.loadtxt(folder + 'RMS_Sta_' + str(Trail-1) + str(i) + '.txt')
            Con = np.loadtxt(folder + 'Controller_' + str(Trail-1) + str(i) + '.txt')
            opt_sta[i, 0] = RMSSta0[0]
            opt_sta[i, 1] = RMSSta0[1]
            opt_con[i, :num_par-2] = Con[:num_par-2]*Scaling
            opt_con[i, num_par-2:] = Con[num_par-2:]
            
        sta_ind = np.where(opt_sta[:, 1]==0)[0]
        rms_ind = np.argsort(opt_sta[sta_ind, 0])
        con_sort = opt_con[sta_ind[rms_ind], :]
        
        aver_numb = len(np.where(((opt_sta[sta_ind[rms_ind], 0] - opt_sta[sta_ind[rms_ind][0], 0])/opt_sta[sta_ind[rms_ind][0], 0]) < 0.01)[0])
        
        ConAve = np.mean(con_sort[:aver_numb, :] ,axis=0)
        ConStd = np.std(con_sort[:aver_numb, :] ,axis=0)
        
        width = 0.35
        ind1 = np.arange(4)    # the x locations for the groups
        ind2 = np.arange(2)
        
        fig1 = plt.figure(figsize=(6, 9))
        ax1 = fig1.add_subplot(3,1,1)
        plt.ylabel('N*m/rad (N*m*s/rad)')
        plt.xticks(ind1, ('Kp_aa', 'Kd_aa', 'Kp_ha', 'Kd_ha'))
        ax2 = fig1.add_subplot(3,1,2)
        plt.ylabel('N*m/rad (N*m*s/rad)')
        plt.xticks(ind1, ('Kp_hh', 'Kd_hh', 'Kp_ah', 'Kd_ah'))
        ax3 = fig1.add_subplot(3,1,3)
        plt.ylabel('Degree')
        plt.xticks(ind2, ('Ref_a', 'Ref_h'))

        ax1.bar(ind1, ConAve[[0, 2, 1, 3]], width, yerr=ConStd[[0, 2, 1, 3]], error_kw=dict(elinewidth=4, ecolor='red'))
        ax2.bar(ind1, ConAve[[5, 7, 4, 6]], width, yerr=ConStd[[5, 7, 4, 6]], error_kw=dict(elinewidth=4, ecolor='red'))
        ax3.bar(ind2, ConAve[[8, 9]], width, yerr=ConStd[[8, 9]], error_kw=dict(elinewidth=4, ecolor='red'))
        
        k = sta_ind[rms_ind[0]]
        
        time = np.linspace(start_time, end_time, num=(end_nodes-start_nodes))
        
        X_String = folderE + 'JointsGFL000' + str(Trail+1) + '.txt'
        Acc_String = folderE + 'SwayL000' + str(Trail+1) + '.txt'
        TraBestStr = folder + 'TrajectoryResult_' + str(Trail-1) + str(k) + '.txt'
        
        x_meas = np.loadtxt(X_String, usecols=(1, 2, 3, 4))*3.1416/180.0
        acc_meas = np.loadtxt(Acc_String, usecols=(3))
        TraBest = np.loadtxt(TraBestStr, usecols=(0, 1, 2, 3))
        
        fig2 = plt.figure(figsize = (12,8))
        ax4 = fig2.add_subplot(3,1, 1)
        plt.ylabel('Ankle Angle (degree)')
        ax5 = fig2.add_subplot(3,1, 2)
        plt.ylabel('Hip Angle (degree)')
        ax6 = fig2.add_subplot(3,1, 3)
        plt.ylabel('Accel Pert. (m/s^2)')
        plt.xlabel('Time (s)')
        
        p1 = ax4.plot(time, x_meas[start_nodes:end_nodes, 0], 'r-', linewidth=2.0,  label = 'Measurement Ankle')
        p3 = ax5.plot(time, x_meas[start_nodes:end_nodes, 1], 'c-', linewidth=2.0,  label = 'Measurement Hip')
        p5 = ax6.plot(time, acc_meas[start_nodes:end_nodes], 'k-', linewidth=2.0,  label = 'Pert. Accel')
        
        if num_repeat == 0:
            p2 = ax4.plot(time, TraBest[start_nodes-8000:end_nodes-8000, 0],
                          '--', linewidth=2.0, label = 'Optimal Fit Ankle'+ str(0))
            p4 = ax5.plot(time, TraBest[start_nodes-8000:end_nodes-8000, 1],
                          '--', linewidth=2.0, label = 'Optimal Fit Hip'+ str(0))
        
        for r in range(num_repeat):
            p2 = ax4.plot(time, TraBest[r*num_nodes + start_nodes-8000:r*num_nodes + end_nodes-8000, 0],
                          '--', linewidth=2.0, label = 'Optimal Fit Ankle'+ str(r))
            p4 = ax5.plot(time, TraBest[r*num_nodes + start_nodes-8000:r*num_nodes + end_nodes-8000, 1],
                          '--', linewidth=2.0, label = 'Optimal Fit Hip'+ str(r))
        
        ax5.legend()
        ax4.legend()
        
        _, _, r_value1, _, _ = stats.linregress(x_meas[start_nodes:end_nodes, 0], 
                                                TraBest[:end_nodes-start_nodes, 0])
        _, _, r_value2, _, _ = stats.linregress(x_meas[start_nodes:end_nodes, 1], 
                                                TraBest[:end_nodes-start_nodes, 1])
        
        rms = np.sqrt(sum((x_meas[start_nodes:end_nodes, 0] - TraBest[:end_nodes-start_nodes, 0])**2 
                          + (x_meas[start_nodes:end_nodes, 1] - TraBest[:end_nodes-start_nodes, 1])**2)/num_nodes)
        
        if StoreSign == 'True':
            fig1.savefig(StorePath + 'Con_RMS_Sta.png')
            fig2.savefig(StorePath + 'Best_Identified_Trajectory.png')
            
            with open(StorePath + 'Best_Con.txt','w') as Outfile:
                StringP = ""
                for m in range(0,num_par):
                    StringP += str(ConAve[m])
                    StringP += " "
                StringP += "\n"
                Outfile.write(StringP)
                
            with open(StorePath + 'Best_Con_Std.txt','w') as Outfile:
                StringP = ""
                for m in range(0,num_par):
                    StringP += str(ConStd[m])
                    StringP += " "
                StringP += "\n"
                Outfile.write(StringP)
                
            with open(StorePath + 'Best_RMS_R.txt','w') as Outfile:
                StringP = ""
                StringP += str(rms)
                StringP += "\n"
                StringP += str(r_value1)
                StringP += "\n"
                StringP += str(r_value2)
                StringP += "\n"
                Outfile.write(StringP)
                
            with open(StorePath + 'Sort_Con.txt','w') as Outfile:
                StringP = ""
                for m in range(len(con_sort[:, 0])):
                    for n in range(0,num_par):
                        StringP += str(con_sort[m, n])
                        StringP += " "
                    StringP += "\n"
                Outfile.write(StringP)
                
            with open(StorePath + 'Best_Traj.txt','w') as Outfile:
                StringP = ""
                for m in range(0,num_nodes*num_repeat):
                    for n in range(0,num_states):
                        StringP += str(TraBest[m, n])
                        StringP += " "
                    StringP += "\n"
                Outfile.write(StringP)