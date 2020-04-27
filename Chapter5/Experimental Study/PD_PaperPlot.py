#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 11:56:12 2019

Result Analysis of the Single Pendulum Optimization.

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
#from pylab import setp

St = 0
Ed = 20

num_nodes = 5001
interval = 0.02
duration = (num_nodes-1)*interval

start_nodes = 8000

num_states = 4
num_cons = 4
Scaling = 50.0
integration_method = 'midpoint'
max_delay_time = 0.12
half_state = int(num_states/2)

num_par = 6
num_subj = 6

Mean_Gains = np.zeros((num_subj, 2, num_par))
Best_Gains = np.zeros((num_subj, 2, num_par))
STD_Gains = np.zeros((num_subj, 2, num_par))
Fit_Info = np.zeros((num_subj, 2, 3))

heights = [1.60, 1.80, 1.78, 1.79, 1.65, 1.75, 1.63]

g = 9.8

for sj in range(num_subj):

    subj = 3 + sj
        
    for tr in range(2):
        store_path = 'PD/Analysis_Paper/Subj0'+str(subj)+'/Pert'+str(1+tr)+'_GFL_SPDR_Eps2_Noise025/'
        
        Mean_Gains[sj, tr, :] = np.loadtxt(store_path + 'Best_Con.txt')
        Best_Gains[sj, tr, :] = np.loadtxt(store_path + 'Sort_Con.txt')[0, :]
        STD_Gains[sj, tr, :] = np.loadtxt(store_path + 'Best_Con_Std.txt')
        Fit_Info[sj, tr, :] = np.loadtxt(store_path + 'Best_RMS_R.txt')
        
        Para = np.loadtxt('Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt')
        
        mass = Para[3] + Para[4]
        height = heights[int(subj -3)]
        
        mass_normal = 70
        height_normal = 1.75
        
        
        Kp_normalization = mass*height/(mass_normal*height_normal)
        Kd_normalization = (mass*height/np.sqrt(g/height))/(mass_normal*height_normal/np.sqrt(g/height_normal))
        
        Mean_Gains[sj, tr, 0] = Mean_Gains[sj, tr, 0]/Kp_normalization
        Mean_Gains[sj, tr, 1] = Mean_Gains[sj, tr, 1]/Kd_normalization
        Mean_Gains[sj, tr, 2] = Mean_Gains[sj, tr, 2]/Kp_normalization
        Mean_Gains[sj, tr, 3] = Mean_Gains[sj, tr, 3]/Kd_normalization
        
        Best_Gains[sj, tr, 0] = Best_Gains[sj, tr, 0]/Kp_normalization
        Best_Gains[sj, tr, 1] = Best_Gains[sj, tr, 1]/Kd_normalization
        Best_Gains[sj, tr, 2] = Best_Gains[sj, tr, 2]/Kp_normalization
        Best_Gains[sj, tr, 3] = Best_Gains[sj, tr, 3]/Kd_normalization
        
        STD_Gains[sj, tr, 0] = STD_Gains[sj, tr, 0]/Kp_normalization
        STD_Gains[sj, tr, 1] = STD_Gains[sj, tr, 1]/Kd_normalization
        STD_Gains[sj, tr, 2] = STD_Gains[sj, tr, 2]/Kp_normalization
        STD_Gains[sj, tr, 3] = STD_Gains[sj, tr, 3]/Kd_normalization
        
with open('PD/Analysis_Paper/Lowest_RMS.txt', 'w') as outfile:
    StringP = ''
    for i in range(num_subj):
        for j in range(2):
            StringP += str(Fit_Info[i, j, 0])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open('PD/Analysis_Paper/Highest_R1.txt', 'w') as outfile:
    StringP = ''
    for i in range(num_subj):
        for j in range(2):
            StringP += str(Fit_Info[i, j, 1])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open('PD/Analysis_Paper/Highest_R2.txt', 'w') as outfile:
    StringP = ''
    for i in range(num_subj):
        for j in range(2):
            StringP += str(Fit_Info[i, j, 2])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)

ind1 = np.arange(2)
ind2 = np.arange(2)

SMALL_SIZE = 12
MEDIUM_SIZE = 12


fig1 = plt.figure(figsize=(7.5,7))

ind_ankle = np.array([0, 1])
ind_hip = np.array([2, 3])
ind_td_ref = np.array([4, 5])

ax1 = fig1.add_subplot(3, 1, 1)
#plt.title('Mean Gains')
ax1.plot([-2 -2], [-2 -2],'b-', label='Perturbation Trial 1')
ax1.plot([-2 -2], [-2 -2],'g-', label='Perturbation Trial 2')
ax1.plot([ind1[0]-1, ind1[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)
bp1_1 = ax1.boxplot(Mean_Gains[:, 0, ind_ankle], positions = ind1-0.2, widths = 0.2, showfliers=False)
bp2_1 = ax1.boxplot(Mean_Gains[:, 1, ind_ankle], positions = ind1+0.2, widths = 0.2, showfliers=False)

ax1.set_xlim([-1, len(ind1)])
ax1.set_xticklabels(['K11', 'B11'])
ax1.set_xticks(list(ind1))
plt.ylabel('Ankle joint gains\n(Nm/rad, Nms/rad)')
plt.legend()

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize

ax2 = fig1.add_subplot(3, 1, 2)
ax2.plot([ind1[0]-1, ind1[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)
bp1_2 = ax2.boxplot(Mean_Gains[:, 0, ind_hip], positions = ind1-0.2, widths = 0.2, showfliers=False)
bp2_2 = ax2.boxplot(Mean_Gains[:, 1, ind_hip], positions = ind1+0.2, widths = 0.2, showfliers=False)

plt.ylabel('Hip joint gains\n(Nm/rad, Nms/rad)')

ax2.set_xlim([-1, len(ind1)])
ax2.set_xticklabels(['K22', 'B22'])
ax2.set_xticks(list(ind1))

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize

ax3 = fig1.add_subplot(3, 1, 3)
ax3.plot([ind2[0]-1, ind2[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)
bp1_3 = ax3.boxplot(Mean_Gains[:, 0, ind_td_ref]*180/np.pi, positions = ind2-0.2, widths = 0.2, showfliers=False)
bp2_3 = ax3.boxplot(Mean_Gains[:, 1, ind_td_ref]*180/np.pi, positions = ind2+0.2, widths = 0.2, showfliers=False)

plt.ylabel('Ref angles (degree)')
plt.xticks(ind2, ('Ref1', 'Ref2'))

ax3.set_xlim([-1, len(ind2)])
ax3.set_xticklabels(['r1', 'r2'])
ax3.set_xticks(list(ind2))

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize


plt.setp(bp1_1['boxes'], color='blue')
plt.setp(bp1_1['caps'], color='blue')
plt.setp(bp1_2['boxes'], color='blue')
plt.setp(bp1_2['caps'], color='blue')
plt.setp(bp1_3['boxes'], color='blue')
plt.setp(bp1_3['caps'], color='blue')

plt.setp(bp2_1['boxes'], color='green')
plt.setp(bp2_1['caps'], color='green')
plt.setp(bp2_2['boxes'], color='green')
plt.setp(bp2_2['caps'], color='green')
plt.setp(bp2_3['boxes'], color='green')
plt.setp(bp2_3['caps'], color='green')


fig2 = plt.figure(figsize=(7.5,7))
ax1 = fig2.add_subplot(3, 1, 1)
#plt.title('Best Gains')
ax1.plot([-2 -2], [-2 -2],'b-', label='Perturbation Trial 1')
ax1.plot([-2 -2], [-2 -2],'g-', label='Perturbation Trial 2')
plt.ylabel('Ankle joint gains\n(Nm/rad, Nms/rad)')
plt.legend()
plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
ax2 = fig2.add_subplot(3, 1, 2)
plt.ylabel('Hip joint gains\n(Nm/rad, Nms/rad)')
plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
ax3 = fig2.add_subplot(3, 1, 3)
plt.ylabel('Ref angles (deg)')
plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize

ind_ankle = np.array([0, 1])
ind_hip = np.array([2, 3])
ind_td_ref = np.array([4, 5])

ax1.plot([ind1[0]-1, ind1[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)
ax2.plot([ind1[0]-1, ind1[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)
ax3.plot([ind2[0]-1, ind2[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)

bp1_1 = ax1.boxplot(Best_Gains[:, 0, ind_ankle], positions = ind1-0.2, widths = 0.2, showfliers=False)
bp1_2 = ax2.boxplot(Best_Gains[:, 0, ind_hip], positions = ind1-0.2, widths = 0.2, showfliers=False)
bp1_3 = ax3.boxplot(Best_Gains[:, 0, ind_td_ref]*180/np.pi, positions = ind2-0.2, widths = 0.2, showfliers=False)

bp2_1 = ax1.boxplot(Best_Gains[:, 1, ind_ankle], positions = ind1+0.2, widths = 0.2, showfliers=False)
bp2_2 = ax2.boxplot(Best_Gains[:, 1, ind_hip], positions = ind1+0.2, widths = 0.2, showfliers=False)
bp2_3 = ax3.boxplot(Best_Gains[:, 1, ind_td_ref]*180/np.pi, positions = ind2+0.2, widths = 0.2, showfliers=False)

ax1.set_xlim([-1, len(ind1)])
ax1.set_xticklabels(['K11', 'B11'])
ax1.set_xticks(list(ind1))

ax2.set_xlim([-1, len(ind1)])
ax2.set_xticklabels(['K22', 'B22'])
ax2.set_xticks(list(ind1))

ax3.set_xlim([-1, len(ind2)])
ax3.set_xticklabels(['Ref1', 'Ref2'])
ax3.set_xticks(list(ind2))

plt.setp(bp1_1['boxes'], color='blue')
plt.setp(bp1_1['caps'], color='blue')
plt.setp(bp1_2['boxes'], color='blue')
plt.setp(bp1_2['caps'], color='blue')
plt.setp(bp1_3['boxes'], color='blue')
plt.setp(bp1_3['caps'], color='blue')

plt.setp(bp2_1['boxes'], color='green')
plt.setp(bp2_1['caps'], color='green')
plt.setp(bp2_2['boxes'], color='green')
plt.setp(bp2_2['caps'], color='green')
plt.setp(bp2_3['boxes'], color='green')
plt.setp(bp2_3['caps'], color='green')