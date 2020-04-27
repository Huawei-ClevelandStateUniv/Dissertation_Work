#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 20 19:30:33 2020

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt


folder_1 = 'F1_S0/'
folder_2 = 'F1_S1/'
folder_3 = 'F2_S0/'
folder_4 = 'F2_S1/'

ntrial = 20

rms = np.zeros((ntrial, 4))
sta = np.zeros((ntrial, 4)) - 1

for k in range(ntrial):
    rms[k, 0] = np.loadtxt(folder_1 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[0]
    sta[k, 0] = np.loadtxt(folder_1 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[1]
    
    rms[k, 1] = np.loadtxt(folder_2 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[0]
    sta[k, 1] = np.loadtxt(folder_2 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[1]
    if k < 10:
        rms[k, 2] = np.loadtxt(folder_3 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[0]
        sta[k, 2] = np.loadtxt(folder_3 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[1]
    
        rms[k, 3] = np.loadtxt(folder_4 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[0]
        sta[k, 3] = np.loadtxt(folder_4 + 'RMS_Sta_Scaling1.0nodes251impedance_50HZ_' + str(k) + '.txt')[1]

best_ind = np.zeros(4, dtype=int)

for j in range(4):    
    sta_ind = np.where(sta[:, j] == 0)[0]
    rms_ind = np.argsort(rms[sta_ind, j])
    
    best_ind[j] = sta_ind[rms_ind][0]
    
con_1 = np.loadtxt(folder_1 + 'Impedance_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[0]) + '.txt')
con_2 = np.loadtxt(folder_2 + 'Impedance_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[1]) + '.txt')
con_3 = np.loadtxt(folder_3 + 'Impedance_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[2]) + '.txt')
con_4 = np.loadtxt(folder_4 + 'Impedance_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[3]) + '.txt')

traj_1 = np.loadtxt(folder_1 + 'TrajectoryResult_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[0]) + '.txt')
traj_2 = np.loadtxt(folder_2 + 'TrajectoryResult_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[1]) + '.txt')
traj_3 = np.loadtxt(folder_3 + 'TrajectoryResult_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[2]) + '.txt')
traj_4 = np.loadtxt(folder_4 + 'TrajectoryResult_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[3]) + '.txt')

ref_traj_1 = np.loadtxt(folder_1 + 'TrajRef_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[0]) + '.txt')
ref_traj_2 = np.loadtxt(folder_2 + 'TrajRef_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[1]) + '.txt')
ref_traj_3 = np.loadtxt(folder_4 + 'TrajRef_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[2]) + '.txt')
ref_traj_4 = np.loadtxt(folder_4 + 'TrajRef_Scaling1.0nodes251impedance_50HZ_' + str(best_ind[3]) + '.txt')

x_table = np.array([0, 1])

fig1 = plt.figure(figsize=(7, 6))
ax1 = fig1.add_subplot(2,2,1)
ax1.bar(x_table, con_1, width=0.3, label = 'Participant 2 at 0.8 m/s')
plt.ylabel('Impedance parameters\n(Nm/rad, Nms/rad)')
plt.xticks(x_table, ['', ''])
plt.ylim([0, 230])
plt.legend()

ax2 = fig1.add_subplot(2,2,2)
ax2.bar(x_table, con_2, width=0.3, label = 'Participant 2 at 1.2 m/s')
plt.xticks(x_table, ['', ''])
plt.ylim([0, 230])
plt.yticks([0, 50, 100, 150, 200], [' ', ' ', ' ',  ' ', ' '])
plt.legend()

ax3 = fig1.add_subplot(2,2,3)
ax3.bar(x_table, con_3, width=0.3, label = 'Participant 3 at 0.8 m/s')
plt.ylabel('Impedance parameters\n(Nm/rad, Nms/rad)')
plt.xticks(x_table, ['Kp', 'kd'])
plt.ylim([0, 230])
plt.legend()

ax4 = fig1.add_subplot(2,2,4)
ax4.bar(x_table, con_4, width=0.3, label = 'Participant 3 at 1.2 m/s')
plt.xticks(x_table, ['Kp', 'kd'])
plt.ylim([0, 230])
plt.yticks([0, 50, 100, 150, 200], [' ', ' ', ' ',  ' ', ' '])
plt.legend()







#gender = 'F'
#start_nodes = 1000
#num_nodes = 251
#duration = 5.0
#
#S = 1
#T = 1
#
#selected_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/selected_data/'
#                  + gender + str(S) + '_S' + str(T) + '/')
#
#x_meas = np.loadtxt(selected_path + 'Motion_2dmodel_30000_40000_50HZ.txt')[start_nodes:start_nodes+num_nodes, :]
#
#if np.mean(x_meas[:, 5]) > np.pi/2:
#    x_meas[:, 5] -= np.pi
#elif np.mean(x_meas[:, 5]) < -np.pi/2:
#    x_meas[:, 5] += np.pi
#    
#if np.mean(x_meas[:, 8]) > np.pi/2:
#    x_meas[:, 8] -= np.pi
#elif np.mean(x_meas[:, 8]) < -np.pi/2:
#    x_meas[:, 8] += np.pi
#
#time_plot = np.linspace(0, duration, num_nodes)
#
#fig2 = plt.figure(figsize=(8, 6))
#ax1 = fig2.add_subplot(3, 2, 1)
#plt.ylabel('Hip (rad)', fontsize=14)
#plt.title('Left Leg', fontsize=14)
#ax1.plot(time_plot, x_meas[:num_nodes, 3], 'r-', linewidth=2.5, label='Experimental motion')
#ax1.plot(time_plot, traj_Ph1[:num_nodes, 3], '-')
#ax1.plot(time_plot, traj_Ph2[:num_nodes, 3], '-')
#ax1.plot(time_plot, traj_Ph4[:num_nodes, 3], '-')
#
#ax2 = fig2.add_subplot(3, 2, 2)
#plt.title('Right Leg', fontsize=14)
#ax2.plot(time_plot, x_meas[:num_nodes, 6], 'r-', linewidth=2.5, label='Experiment data')
#ax2.plot(time_plot, traj_Ph1[:num_nodes, 6], '-', label='One phase')
#ax2.plot(time_plot, traj_Ph2[:num_nodes, 6], '-', label='Two phase')
#ax2.plot(time_plot, traj_Ph4[:num_nodes, 6], '-', label='Four phase')
#plt.legend(bbox_to_anchor=(0.65, 1), loc = 3 )
#
#ax3 = fig2.add_subplot(3, 2, 3)
#plt.ylabel('Knee (rad)', fontsize=14)
#ax3.plot(time_plot, x_meas[:num_nodes, 4], 'r-', linewidth=2.5, label='Experiment data')
#ax3.plot(time_plot, traj_Ph1[:num_nodes, 4], '-')
#ax3.plot(time_plot, traj_Ph2[:num_nodes, 4], '-')
#ax3.plot(time_plot, traj_Ph4[:num_nodes, 4], '-')
#
#ax4 = fig2.add_subplot(3, 2, 4)
#ax4.plot(time_plot, x_meas[:num_nodes, 7], 'r-', linewidth=2.5, label='Experiment data')
#ax4.plot(time_plot, traj_Ph1[:num_nodes, 7], '-')
#ax4.plot(time_plot, traj_Ph2[:num_nodes, 7], '-')
#ax4.plot(time_plot, traj_Ph4[:num_nodes, 7], '-')
#
#ax5 = fig2.add_subplot(3, 2, 5)
#plt.ylabel('Ankle (rad)', fontsize=14)
#plt.xlabel('Time (s)', fontsize=14)
#ax5.plot(time_plot, x_meas[:num_nodes, 5], 'r-', linewidth=2.5, label='Experiment data')
#ax5.plot(time_plot, traj_Ph1[:num_nodes, 5], '-')
#ax5.plot(time_plot, traj_Ph2[:num_nodes, 5], '-')
#ax5.plot(time_plot, traj_Ph4[:num_nodes, 5], '-')
#
#ax6 = fig2.add_subplot(3, 2, 6)
#plt.xlabel('Time (s)', fontsize=14)
#ax6.plot(time_plot, x_meas[:num_nodes, 8], 'r-', linewidth=2.5, label='Experiment data')
#ax6.plot(time_plot, traj_Ph1[:num_nodes, 8], '-')
#ax6.plot(time_plot, traj_Ph2[:num_nodes, 8], '-')
#ax6.plot(time_plot, traj_Ph4[:num_nodes, 8], '-')
#
#fig3 = plt.figure(figsize=(4, 4))
#plt.plot(ref_traj_Ph1)
#plt.plot(ref_traj_Ph2)
#plt.plot(ref_traj_Ph4)





 


