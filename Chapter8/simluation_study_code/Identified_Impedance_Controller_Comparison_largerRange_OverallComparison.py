#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 28 00:42:28 2020

Compare Identified Control Gains

@author: huawei
"""
import numpy as np
import matplotlib.pyplot as plt

gender = 'F'
S = 1
T = 1

Scaling = 1
mass = 60

simulation_data_path = ('Simulation_Study_Data/Simulation_2dData_Wmt100Wms0.005Wts1e-05Wtm1e-07Wtp0.1/'
          + gender + str(S) + '_S' + str(T) + '_N501_JC2_Phase4GoodInit2/')


# load simulation impedance parameters
imped_sim = np.loadtxt(simulation_data_path + 'Simulated_imped.txt')
baseline_sim = np.loadtxt(simulation_data_path + 'Simulated_torques_pero.txt')[:50, :]

num_inst = 10

con_means = np.zeros((num_inst, len(imped_sim)))
con_std = np.zeros((num_inst, len(imped_sim)))

con_means_all = np.zeros((5, len(imped_sim)))
con_std_all = np.zeros((5, len(imped_sim)))

baseline_knee_mean = np.zeros((len(baseline_sim[:, 0]), 5))
baseline_ankle_mean = np.zeros((len(baseline_sim[:, 1]), 5))

baseline_knee_std = np.zeros((len(baseline_sim[:, 0]), 5))
baseline_ankle_std = np.zeros((len(baseline_sim[:, 1]), 5))

baseline_knee_mean[:, 0] = baseline_sim[:, 0]
baseline_ankle_mean[:, 0] = baseline_sim[:, 1]

baseline_knee = np.zeros((len(baseline_sim[:, 0]), 10))
baseline_ankle = np.zeros((len(baseline_sim[:, 0]), 10))

baseline_knee_all = np.zeros((len(baseline_sim[:, 0]), 10))
baseline_ankle_all = np.zeros((len(baseline_sim[:, 0]), 10))

con_means_all[0, :] = np.loadtxt(simulation_data_path + 'Simulated_imped.txt')
con_means_all[1, :] = np.loadtxt('Simulation_Study_Data/Simulation_Id_New/Simulated_imped.txt')

baseline_knee_mean[:, 1] = np.loadtxt('Simulation_Study_Data/Simulation_Id_New/Simulated_torques_pero.txt')[:50, 0]
baseline_ankle_mean[:, 1] = np.loadtxt('Simulation_Study_Data/Simulation_Id_New/Simulated_torques_pero.txt')[:50, 1]

for var in range(2, 5):
    for inst in range(0, 10):
        if var == 2:
            store_path = ('Simulation_Study_Data/Simulation_Id_SN_'+str(inst)+'_LargerRange/')
        elif var == 3:
            store_path = ('Simulation_Study_Data/Simulation_Id_10Percent_Parameter_'+str(inst)+'_LargerRange/')
        elif var == 4:
            store_path = ('Simulation_Study_Data/Simulation_Id_10Percent_footsize_'+str(inst)+'_LargerRange/')
 
        ntrial = 10
        con_close = np.zeros((ntrial, len(imped_sim)))
        rms = np.zeros(ntrial)
        sta = np.zeros(ntrial)
        
        for k in range(ntrial):
            con_close[k, :] = np.loadtxt(store_path + 'Impedance_Scaling' + str(Scaling) + 'HStiff_SmoothGround_50HZ_'+str(k)+'.txt')
            rms[k] = np.loadtxt(store_path + 'RMS_Sta_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(k) + '.txt')[0]
            sta[k] = np.loadtxt(store_path + 'RMS_Sta_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(k) + '.txt')[1]
            
            baseline_knee[:, k] = np.loadtxt(store_path + 'Normal_Torques_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(k) + '.txt')[:, 0]
            baseline_ankle[:, k] = np.loadtxt(store_path + 'Normal_Torques_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(k) + '.txt')[:, 1]
        
        sta_ind = np.where(sta==0)[0]
        rms_ind = np.argsort(rms[sta_ind])
        
        ind_similar = 0 #np.where(rms[sta_ind[rms_ind]] < rms[sta_ind[rms_ind[0]]]*1.05)[0]
        best_ind = sta_ind[rms_ind[ind_similar]]
        
        con_select = con_close[best_ind, :]
        
        baseline_knee_select = baseline_knee[:, best_ind]
        baseline_ankle_select = baseline_ankle[:, best_ind]
        
        con_means[inst, :] = con_select #np.mean(con_select, axis=0)
        #con_std[inst, :] = np.std(con_select, axis=0)
        baseline_knee_all[:, inst] = baseline_knee_select
        baseline_ankle_all[:, inst] = baseline_ankle_select
        
    con_means_all[var, :] = np.mean(con_means, axis = 0)
    con_std_all[var, :] = np.std(con_means, axis = 0)
    
    baseline_knee_mean[:, var] = np.mean(baseline_knee_all, axis = 1)
    baseline_ankle_mean[:, var] = np.mean(baseline_ankle_all, axis = 1)
    
    baseline_knee_std[:, var] = np.std(baseline_knee_all, axis = 1)
    baseline_ankle_std[:, var] = np.std(baseline_ankle_all, axis = 1)
    
    
AbosluteError = np.zeros((4, len(con_means_all[0, :])))
RelativeError = np.zeros((4, len(con_means_all[0, :])))

for e in range(4):
    AbosluteError[e, :] = con_means_all[e+1, :] - con_means_all[0, :]
    RelativeError[e, :] = (con_means_all[e+1, :] - con_means_all[0, :])/con_means_all[0, :]*100
    
MEDIUM_SIZE = 11
width = 0.35

ind_g = 0
ind_id = np.array([1, 2, 3, 4])
labels = ['True value', 'Test condition 1', 'Test condition 2', 'Test condition 3', 'Test condition 4']
#
ind_all = np.array([0, 1, 2, 3, 4])

fig1 = plt.figure(figsize=(12, 8))

ax1 = fig1.add_subplot(4,2,1)
plt.ylabel(' Kp in phase 1\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 25])
ax2 = fig1.add_subplot(4,2,2)
plt.ylabel('Kd in phase 1\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 6.5])
ax3 = fig1.add_subplot(4,2,3)
plt.ylabel('Kp in phase 2\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 25])
ax4 = fig1.add_subplot(4,2,4)
plt.ylabel('Kd in phase 2\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 6.5])
ax5 = fig1.add_subplot(4,2,5)
plt.ylabel('Kp in phase 3\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 25])
ax6 = fig1.add_subplot(4,2,6)
plt.ylabel('Kd in phase 3\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 6.5])
ax7 = fig1.add_subplot(4,2,7)
plt.ylabel('Kp in phase 4\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 25])

ax7.bar(ind_id, con_means_all[1:, 6], width, yerr=con_std_all[1:, 6], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax7.plot([0, len(ind_id)], [con_means_all[0, 6], con_means_all[0, 6]] , '--',  color = 'C1', zorder=0)
ax7.bar(ind_g, con_means_all[0, 6], width, alpha=0.8, color = 'C1')
ax7.set_xticks(ind_all)
ax7.set_xticklabels(labels)
plt.xticks(rotation=-30)

ax8 = fig1.add_subplot(4,2,8)
plt.ylabel('Kd in phase 4\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 6.5])

ax1.bar(ind_id, con_means_all[1:, 0], width, yerr=con_std_all[1:, 0], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax1.plot([0, len(ind_id)], [con_means_all[0, 0], con_means_all[0, 0]] , '--',  color = 'C1', zorder=0)
ax1.bar(ind_g, con_means_all[0, 0], width, alpha=0.8, color = 'C1')
ax1.set_xticks(ind_all)
ax1.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax2.bar(ind_id, con_means_all[1:, 1], width, yerr=con_std_all[1:, 1], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax2.plot([0, len(ind_id)], [con_means_all[0, 1], con_means_all[0, 1]] , '--',  color = 'C1', zorder=0)
ax2.bar(ind_g, con_means_all[0, 1], width, alpha=0.8, color = 'C1')
ax2.set_xticks(ind_all)
ax2.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax3.bar(ind_id, con_means_all[1:, 2], width, yerr=con_std_all[1:, 2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax3.plot([0, len(ind_id)], [con_means_all[0, 2], con_means_all[0, 2]] , '--',  color = 'C1', zorder=0)
ax3.bar(ind_g, con_means_all[0, 2], width, alpha=0.8, color = 'C1')
ax3.set_xticks(ind_all)
ax3.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax4.bar(ind_id, con_means_all[1:, 3], width, yerr=con_std_all[1:, 3], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax4.plot([0, len(ind_id)], [con_means_all[0, 3], con_means_all[0, 3]] , '--',  color = 'C1', zorder=0)
ax4.bar(ind_g, con_means_all[0, 3], width, alpha=0.8, color = 'C1')
ax4.set_xticks(ind_all)
ax4.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax5.bar(ind_id, con_means_all[1:, 4], width, yerr=con_std_all[1:, 4], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax5.plot([0, len(ind_id)], [con_means_all[0, 4], con_means_all[0, 4]] , '--',  color = 'C1', zorder=0)
ax5.bar(ind_g, con_means_all[0, 4], width, alpha=0.8, color = 'C1')
ax5.set_xticks(ind_all)
ax5.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax6.bar(ind_id, con_means_all[1:, 5], width, yerr=con_std_all[1:, 5], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax6.plot([0, len(ind_id)], [con_means_all[0, 5], con_means_all[0, 5]] , '--',  color = 'C1', zorder=0)
ax6.bar(ind_g, con_means_all[0, 5], width, alpha=0.8, color = 'C1')
ax6.set_xticks(ind_all)
ax6.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax8.bar(ind_id, con_means_all[1:, 7], width, yerr=con_std_all[1:, 7], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax8.plot([0, len(ind_id)], [con_means_all[1:, 7], con_means_all[1:, 7]] , '--',  color = 'C1', zorder=0)
ax8.bar(ind_g, con_means_all[1:, 7], width, alpha=0.8, color = 'C1')
ax8.set_xticks(ind_all)
ax8.set_xticklabels(labels)
plt.xticks(rotation=-30)


fig2 = plt.figure(figsize=(12, 8))
ax1 = fig2.add_subplot(4,2,1)
plt.ylabel(' Kp in phase 1\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 550])
ax2 = fig2.add_subplot(4,2,2)
plt.ylabel('Kd in phase 1\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 4.5])
ax3 = fig2.add_subplot(4,2,3)
plt.ylabel('Kp in phase 2\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 550])
ax4 = fig2.add_subplot(4,2,4)
plt.ylabel('Kd in phase 2\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 4.5])
ax5 = fig2.add_subplot(4,2,5)
plt.ylabel('Kp in phase 3\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 550])
ax6 = fig2.add_subplot(4,2,6)
plt.ylabel('Kd in phase 3\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 4.5])
ax7 = fig2.add_subplot(4,2,7)
plt.ylabel('Kp in phase 4\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 550])

ax7.bar(ind_id, con_means_all[1:, 14], width, yerr=con_std_all[1:, 14], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax7.plot([0, len(ind_id)], [con_means_all[0, 14], con_means_all[0, 14]] , '--',  color = 'C1', zorder=0)
ax7.bar(ind_g, con_means_all[0, 14], width, alpha=0.8, color = 'C1')
ax7.set_xticks(ind_all)
ax7.set_xticklabels(labels)
plt.xticks(rotation=-30)

ax8 = fig2.add_subplot(4,2,8)
plt.ylabel('Kd in phase 4\n(Nms/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
plt.ylim([0, 4.5])

ax1.bar(ind_id, con_means_all[1:, 8], width, yerr=con_std_all[1:, 8], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax1.plot([0, len(ind_id)], [con_means_all[0, 8], con_means_all[0, 8]] , '--',  color = 'C1', zorder=0)
ax1.bar(ind_g, con_means_all[0, 8], width, alpha=0.8, color = 'C1')
ax1.set_xticks(ind_all)
ax1.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax2.bar(ind_id, con_means_all[1:, 9], width, yerr=con_std_all[1:, 9], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax2.plot([0, len(ind_id)], [con_means_all[0, 9], con_means_all[0, 9]] , '--',  color = 'C1', zorder=0)
ax2.bar(ind_g, con_means_all[0, 9], width, alpha = 0.8, color = 'C1')
ax2.set_xticks(ind_all)
ax2.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax3.bar(ind_id, con_means_all[1:, 10], width, yerr=con_std_all[1:, 10], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax3.plot([0, len(ind_id)], [con_means_all[0, 10], con_means_all[0, 10]] , '--',  color = 'C1', zorder=0)
ax3.bar(ind_g, con_means_all[0, 10], width, alpha=0.8, color = 'C1')
ax3.set_xticks(ind_all)
ax3.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax4.bar(ind_id, con_means_all[1:, 11], width, yerr=con_std_all[1:, 11], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax4.plot([0, len(ind_id)], [con_means_all[0, 11], con_means_all[0, 11]] , '--',  color = 'C1', zorder=0)
ax4.bar(ind_g, con_means_all[0, 11], width, alpha=0.8, color = 'C1')
ax4.set_xticks(ind_all)
ax4.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax5.bar(ind_id, con_means_all[1:, 12], width, yerr=con_std_all[1:, 12], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax5.plot([0, len(ind_id)], [con_means_all[0, 12], con_means_all[0, 12]] , '--',  color = 'C1', zorder=0)
ax5.bar(ind_g, con_means_all[0, 12], width, alpha=0.8, color = 'C1')
ax5.set_xticks(ind_all)
ax5.set_xticklabels([' ', ' ', ' ', ' ', ' '])

ax6.bar(ind_id, con_means_all[1:, 13], width, yerr=con_std_all[1:, 13], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax6.plot([0, len(ind_id)], [con_means_all[0, 13], con_means_all[0, 13]] , '--',  color = 'C1', zorder=0)
ax6.bar(ind_g, con_means_all[0, 13], width, alpha=0.8, color = 'C1')
ax6.set_xticks([])

ax8.bar(ind_id, con_means_all[1:, 15], width, yerr=con_std_all[1:, 15], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax8.plot([0, len(ind_id)], [con_means_all[0, 15], con_means_all[0, 15]] , '--',  color = 'C1', zorder=0)
ax8.bar(ind_g, con_means_all[0, 15], width, alpha=0.8, color = 'C1')
ax8.set_xticks(ind_all)
ax8.set_xticklabels(labels)
plt.xticks(rotation=-30)



fig3 = plt.figure(figsize=(12, 4))

x_axis = np.linspace(0, 100, 50, dtype='int')

ax1 = fig3.add_subplot(1, 2, 1)
plt.plot(x_axis, baseline_knee_mean[:, 0], 'k-', label = 'Simulation data', linewidth=2)
plt.plot(x_axis, baseline_knee_mean[:, 1], '--', label = 'Test condition 1', linewidth=2)
plt.plot(x_axis, baseline_knee_mean[:, 2], '--', color = 'C2', label = 'Test condition 2', linewidth=2)
plt.plot(x_axis, baseline_knee_mean[:, 3], '--', color = 'C3', label = 'Test condition 3', linewidth=2)
plt.plot(x_axis, baseline_knee_mean[:, 4], '--', color = 'C4', label = 'Test condition 4', linewidth=2)
plt.ylabel('Baseline torques of knee joint\n(Nm)')
plt.xlabel('Data nodes')
plt.legend()

ax2 = fig3.add_subplot(1, 2, 2)

plt.plot(baseline_ankle_mean[:, 0], 'k-', label = 'Simulation data', linewidth=2)
plt.plot(baseline_ankle_mean[:, 1], '--', label = 'Test condition 1', linewidth=2)
plt.plot(baseline_ankle_mean[:, 2], '--', label = 'Test condition 2', linewidth=2)
plt.plot(baseline_ankle_mean[:, 3], '--', label = 'Test condition 3', linewidth=2)
plt.plot(baseline_ankle_mean[:, 4], '--', label = 'Test condition 4', linewidth=2)

plt.ylabel('Baseline torques of ankle joint\n(Nm)')
plt.xlabel('Data nodes')
