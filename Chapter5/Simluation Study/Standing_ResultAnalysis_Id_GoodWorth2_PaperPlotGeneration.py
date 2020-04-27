#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 11:56:12 2019

Result Analysis of the Single Pendulum Optimization.

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy import pi


subj = 4
trial = 1

start_nodes = 8000

Accel_String1 = ('../Experimental Study/Data/2DoF Data Simplify/Subj0'+str(subj)+'/SwayL000'+str(trial+1)+'.txt')
X_String1 = ('ForwardSimulation_GoodWorth/DoublePendulum_N10001Noise_0.txt')
Para_string = ('../Experimental Study/Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt') 

pink_noise1 = np.loadtxt('ForwardSimulation_GoodWorth/pink_noise1_10000_0.txt')
pink_noise2 = np.loadtxt('ForwardSimulation_GoodWorth/pink_noise2_10000_0.txt') 

accel_meas = -np.loadtxt(Accel_String1, usecols=(1))[start_nodes:start_nodes+5000]
time = np.loadtxt(Accel_String1, usecols=(0))[start_nodes:start_nodes+5000]
time = time - time[0]
x_meas = np.loadtxt(X_String1)[:10000, :]
parameter = np.loadtxt(Para_string)

time_motion = np.linspace(0, 100, 10000, endpoint=False)

fig1 = plt.figure(figsize = (6,6))

ax1 = fig1.add_subplot(3,1,1)
plt.ylabel('Peturbation (cm)')
ax2 = fig1.add_subplot(3,1,2)
plt.ylabel('Sensor Noise (deg)')

ax3 = fig1.add_subplot(3,1,3)
plt.ylabel('Simulation Data (deg)')
plt.xlabel('Time (s)')

ax1.plot(time, accel_meas*100)

ax2.plot(time_motion, pink_noise1*180/pi, label = 'sensor noise 1')
ax2.plot(time_motion, pink_noise2*180/pi, label = 'sensor noise 2')
ax2.legend(loc=1, bbox_to_anchor=(1.01, 1.04))

ax3.plot(time_motion, x_meas[:, 0]*180/pi, label = 'Ankle Joint')
ax3.plot(time_motion, x_meas[:, 1]*180/pi, label = 'Hip Joint')
ax3.legend(loc=1, bbox_to_anchor=(1.01, 1.04))



Con_GoodWorth = np.array([152.76, 19.09, 891.07, 286.42, 151.48, 19.09, 251.41,
                              175.03, 50.70, 28.00, 0.09, 0.08])

St = 0
Ed = 10

num_states = 4
num_cons = 4

Scaling = 50.0

num_instance = 10

num_periods = 5

best_con_error = np.zeros((num_periods, len(Con_GoodWorth)+2))

best_con_all = np.zeros((num_periods, num_instance, len(Con_GoodWorth)+2))
mean_con_all = np.zeros((num_periods, len(Con_GoodWorth)+2))
std_con_all = np.zeros((num_periods, len(Con_GoodWorth)+2))

for nn in range(5):

    num_nodes = 501 + nn*1000
    interval = 0.02
    duration = (num_nodes-1)*interval
    
    for mm in range(num_instance):
    
        store_path = 'GoodWorth3_Id_dur'+str(duration)+'Sca'+str(Scaling)+'/Simulation' + str(mm) + '/'
        
        integration_method = 'midpoint'
        max_delay_time = 0.12
        half_state = int(num_states/2)
        
        num_par = 2 + num_states*2 + 2 + 2 
        
        itheta_a = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int)
        itheta_h = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int) + 1
        
        opt_sta = np.zeros((int(Ed - St), 2))
        opt_con = np.zeros((int(Ed - St), num_par))
        
        for k in range(St, Ed):
            opt_sta[k, :] = np.loadtxt(store_path + 'RMS_Sta_'+str(k)+'.txt')[:2]
            opt_con[k, :] = np.loadtxt(store_path + 'Controller_'+str(k)+'.txt')
            
        sta_ind = np.where(opt_sta[:, 1]==0)[0]
        rms_ind = np.argsort(opt_sta[sta_ind, 0])
        con_sort = opt_con[sta_ind[rms_ind], :]
        
    #    fig3 = plt.figure()
    #    index = np.linspace(1, len(sta_ind), len(sta_ind), endpoint=True)
    #    plt.bar(index, opt_sta[sta_ind[rms_ind], 0])
        
        
        if not len(sta_ind) == 0:
            
            #aver_numb = 1 #len(np.where(((opt_sta[sta_ind[rms_ind], 0] - opt_sta[sta_ind[rms_ind][0], 0])/opt_sta[sta_ind[rms_ind][0], 0]) < 0.01)[0])
            best_con_all[nn, mm, 2:] = con_sort[0, :-2]
            best_con_all[nn, mm, 0] = best_con_all[nn, mm, 2] + best_con_all[nn, mm, 4]
            best_con_all[nn, mm, 1] = best_con_all[nn, mm, 3] + best_con_all[nn, mm, 8]
            
    mean_con_all[nn, :] = np.mean(best_con_all[nn, :, :], axis=0)

    std_con_all[nn, :] = np.std(best_con_all[nn, :, :], axis=0)

    mean_con_all[nn, :2 + num_states*2+2] = mean_con_all[nn, :2 + num_states*2+2]*Scaling
    std_con_all[nn, :2 + num_states*2+2] = std_con_all[nn, :2 + num_states*2+2]*Scaling
    
    Kp_Kpas = np.array([Con_GoodWorth[0] + Con_GoodWorth[2], Con_GoodWorth[1] + Con_GoodWorth[6]])
    
    best_con_error[nn, :2] = (mean_con_all[nn, :2] - Kp_Kpas)/Kp_Kpas*100
    best_con_error[nn, 2:] = (mean_con_all[nn, 2:] - Con_GoodWorth)/Con_GoodWorth*100
    

ind_gains = np.arange(len(Con_GoodWorth)+2)
labels_gains = np.array(['K11+Kpas1', 'K22+Kpas2', 'Kpas1', 'Kpas2', 'K11', 'K21', 'B11', 'B21', 'K12', 'K22', 'B12', 'B22', 'Td1', 'Td2'])

best_con_CV = std_con_all[2, :]/mean_con_all[2, :]

rank_errors = np.argsort(abs(best_con_error[2, 2:]))
rank_cv = np.argsort(best_con_CV)

best_con_ER = abs(np.hstack((best_con_error[2, :2], best_con_error[2, 2+rank_errors])))
best_con_CV = best_con_CV[rank_cv]

labels_ER = np.hstack((labels_gains[:2], labels_gains[2+rank_errors]))
labels_CV = labels_gains[rank_cv]

fig2 = plt.figure(figsize = (10,6))

ax1 = fig2.add_subplot(2,1,1)
plt.ylabel('Percentage Bias Error')
plt.xticks(rotation=-15)
ax2 = fig2.add_subplot(2,1,2)
plt.ylabel('CV across 10 random seeds')
plt.xticks(rotation=-15)


width = 0.35

ax1.plot([0, len(ind_gains)-1], [2.5, 2.5], 'k--', zorder=0)

ax1.bar(ind_gains, best_con_ER, width, alpha=0.8, zorder=10)
ax1.set_xticks(ind_gains)
ax1.set_xticklabels(labels_ER)


ax2.plot([0, len(ind_gains)-1], [0.1, 0.1], 'k--', zorder=0)

ax2.bar(ind_gains, best_con_CV, width, alpha=0.8, zorder=10)
ax2.set_xticks(ind_gains)
ax2.set_xticklabels(labels_CV)



MEDIUM_SIZE = 11

ind_g = 0
ind_id = np.array([1, 2, 3, 4, 5])
labels = ['True', '10 s', '30 s', '50 s', '70 s', '90 s']

ind_all = np.arange(len(labels))

fig3 = plt.figure(figsize=(12, 6))
ax1 = fig3.add_subplot(3,2,1)
plt.ylabel('K11 gain\n(Nm/rad)')
plt.ylim(0, 1300)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax2 = fig3.add_subplot(3,2,2)
plt.ylabel('B11 gain\n(Nms/rad)')
plt.ylim(0, 220)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax3 = fig3.add_subplot(3,2,3)
plt.ylabel('K21 gain\n(Nm/rad)')
plt.ylim(0, 400)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax4 = fig3.add_subplot(3,2,4)
plt.ylabel('B21 gain\n(Nms/rad)')
plt.ylim(0, 40)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax5 = fig3.add_subplot(3,2,5)
plt.ylabel('Kpas1 gain\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax6 = fig3.add_subplot(3,2,6)
plt.ylabel('Td1 delay (s)')
plt.ylim(0, 0.12)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax1.plot([0, len(ind_id)], [Con_GoodWorth[2], Con_GoodWorth[2]] , '--', color='C1')
ax1.bar(ind_id, mean_con_all[:, 2+2], width, yerr=std_con_all[:, 2+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax1.bar(ind_g, Con_GoodWorth[2], width, alpha=0.8, color = 'C1')

ax1.set_xticks(ind_all)
ax1.set_xticklabels(labels)
ax2.plot([0, len(ind_id)], [Con_GoodWorth[4], Con_GoodWorth[4]] , '--', color='C1')
ax2.bar(ind_id, mean_con_all[:, 4+2], width, yerr=std_con_all[:, 4+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax2.bar(ind_g, Con_GoodWorth[4], width, alpha=0.8, color = 'C1')
ax2.set_xticks([])
ax3.plot([0, len(ind_id)], [Con_GoodWorth[3], Con_GoodWorth[3]] , '--', color='C1')
ax3.bar(ind_id, mean_con_all[:, 3+2], width, yerr=std_con_all[:, 3+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax3.bar(ind_g, Con_GoodWorth[3], width, alpha=0.8, color = 'C1')
ax3.set_xticks([])
ax4.plot([0, len(ind_id)], [Con_GoodWorth[5], Con_GoodWorth[5]] , '--', color='C1')
ax4.bar(ind_id, mean_con_all[:, 5+2], width, yerr=std_con_all[:, 5+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax4.bar(ind_g, Con_GoodWorth[5], width, alpha=0.8, color='C1')
ax4.set_xticks([])

ax5.plot([0, len(ind_id)], [Con_GoodWorth[0], Con_GoodWorth[0]] , '--', color='C1')
ax5.bar(ind_id, mean_con_all[:, 0+2], width, yerr=std_con_all[:, 0+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax5.bar(ind_g, Con_GoodWorth[0], width, alpha=0.8, color='C1')
ax5.set_xticks([])

ax6.plot([0, len(ind_id)], [Con_GoodWorth[10], Con_GoodWorth[10]] , '--', color='C1')
ax6.bar(ind_id, mean_con_all[:, 10+2], width, yerr=std_con_all[:, 10+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax6.bar(ind_g, Con_GoodWorth[10], width, alpha=0.8, color='C1')
ax6.set_xticks([])


fig4 = plt.figure(figsize=(12, 6))

ax7 = fig4.add_subplot(3,2,1)
plt.ylabel('K22 gain\n(Nm/rad)')
plt.ylim(0, 250)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax8 = fig4.add_subplot(3,2,2)
plt.ylabel('B22 gain\n(Nms/rad)')
plt.ylim(0, 50)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax9 = fig4.add_subplot(3,2,3)
plt.ylabel('K12 gain\n(Nm/rad)')
plt.ylim(0, 400)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax10 = fig4.add_subplot(3,2,4)
plt.ylabel('B12 gain\n(Nms/rad)')
plt.ylim(0, 80)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax11 = fig4.add_subplot(3,2,5)
plt.ylabel('Kpas2 gain\n(Nm/rad)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax12 = fig4.add_subplot(3,2,6)
plt.ylabel('Td2 delay (s)')
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes

ax7.plot([0, len(ind_id)], [Con_GoodWorth[7], Con_GoodWorth[7]] , '--', color='C1')
ax7.bar(ind_id, mean_con_all[:, 7+2], width, yerr=std_con_all[:, 7+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax7.bar(ind_g, Con_GoodWorth[7], width, alpha=0.8, color='C1')
ax7.set_xticks(ind_all)
ax7.set_xticklabels(labels)

ax8.plot([0, len(ind_id)], [Con_GoodWorth[9], Con_GoodWorth[9]] , '--', color='C1')
ax8.bar(ind_id, mean_con_all[:, 9+2], width, yerr=std_con_all[:, 9+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax8.bar(ind_g, Con_GoodWorth[9], width, alpha=0.8, color='C1')
ax8.set_xticks([])

ax9.plot([0, len(ind_id)], [Con_GoodWorth[6], Con_GoodWorth[6]] , '--', color='C1')
ax9.bar(ind_id, mean_con_all[:, 6+2], width, yerr=std_con_all[:, 6+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax9.bar(ind_g, Con_GoodWorth[6], width, alpha=0.8, color='C1')
ax9.set_xticks([])

ax10.plot([0, len(ind_id)], [Con_GoodWorth[8], Con_GoodWorth[8]] , '--', color='C1')
ax10.bar(ind_id, mean_con_all[:, 8+2], width, yerr=std_con_all[:, 8+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax10.bar(ind_g, Con_GoodWorth[8], width, alpha=0.8, color='C1')
ax10.set_xticks([])

ax11.plot([0, len(ind_id)], [Con_GoodWorth[1], Con_GoodWorth[1]] , '--', color='C1')
ax11.bar(ind_id, mean_con_all[:, 1+2], width, yerr=std_con_all[:, 1+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax11.bar(ind_g, Con_GoodWorth[1], width, alpha=0.8, color='C1')
ax11.set_xticks([])

ax12.plot([0, len(ind_id)], [Con_GoodWorth[11], Con_GoodWorth[11]] , '--', color='C1')
ax12.bar(ind_id, mean_con_all[:, 11+2], width, yerr=std_con_all[:, 11+2], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax12.bar(ind_g, Con_GoodWorth[11], width, alpha=0.8, color='C1')
ax12.set_xticks([])

fig5 = plt.figure(figsize=(12, 2))

ax13 = fig5.add_subplot(1,2,1)
plt.ylabel('K11 + Kpas1\n(Nm/rad)')
plt.ylim(0, 1350)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
ax14 = fig5.add_subplot(1,2,2)
plt.ylabel('K22 + Kpas2\n(Nm/rad)')
plt.ylim(0, 350)
plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes

ax13.plot([0, len(ind_id)], [Con_GoodWorth[0] + Con_GoodWorth[2], Con_GoodWorth[0] + Con_GoodWorth[2]] , '--', color='C1')
ax13.bar(ind_id, mean_con_all[:, 0], width, yerr=std_con_all[:, 0], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax13.bar(ind_g, Con_GoodWorth[0] + Con_GoodWorth[2], width, alpha=0.8, color='C1')
ax13.set_xticks(ind_all)
ax13.set_xticklabels(labels)

ax14.plot([0, len(ind_id)], [Con_GoodWorth[1] + Con_GoodWorth[6], Con_GoodWorth[1] + Con_GoodWorth[6]] , '--', color='C1')
ax14.bar(ind_id, mean_con_all[:, 1], width, yerr=std_con_all[:, 1], align='center',
       alpha=0.8,
       ecolor='black',
       capsize=4)
ax14.bar(ind_g, Con_GoodWorth[1] + Con_GoodWorth[6], width, alpha=0.8, color='C1')
ax14.set_xticks([])
