#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 28 14:59:11 2019

# This file show the plot of the normal gait

@author: huawei
"""

import sys
sys.path.append('../')


import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import os
from event_detection import event_detection

j = 0
s = 2
        
if j < 4:
    store_path = 'joint_motion_2d_model/F'+str(j)+'_S'+str(s)
    record_name = 'raw_data/F'+str(j)+'_S'+str(s)+'/record.txt'
    motion_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/mocap_dof.txt'
else:
    store_path = 'joint_motion_2d_model/M'+str(j-4)+'_S'+str(s)
    record_name = 'raw_data/M'+str(j-4)+'_S'+str(s)+'/record.txt'
    motion_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/mocap_dof.txt'

full_dof = store_path +'/joint_motion.txt'
full_mom = store_path +'/joint_moment.txt'
full_grf = store_path +'/grf.txt'

grf = np.loadtxt(full_grf)

time_motion = np.linspace(0, len(grf[:, 0])*0.01, len(grf[:, 0]), endpoint=False)

event_sign = ['C', 'E']
time0, time_index = event_detection(record_name, event_sign)
event_index = []

for m in range(len(event_sign)):
    event_index.append(np.abs(time_motion - (time_index[m]-time0)).argmin())

start_node = event_index[0] + 1500
save_nodes = 3000


motion1 = np.loadtxt(full_dof)[start_node:start_node + save_nodes, :]
moment1 = np.loadtxt(full_mom)[start_node:start_node + save_nodes, :]
grf1 = np.loadtxt(full_grf)[start_node:start_node + save_nodes, :]

index_l = np.zeros_like(grf1[:, 1])
index_r = np.zeros_like(grf1[:, 4])

for i in range(len(grf1[:, 2])):
    if grf1[i, 1] > 100:
        index_l[i] = 1
        
    if grf1[i, 4] > 100:
        index_r[i] = 1
        
def remove_outlayer(index):
    
    average = np.mean(np.diff(index))
    #std = np.std(np.diff(index))
    
    outlayer_index = np.where(np.diff(index) < 0.5*average)[0]
    
    new_index = np.delete(index, outlayer_index)
    
    close_edge_index = np.where(index < 5)[0]
    
    new_index2 = np.delete(new_index, close_edge_index)
    
    if max(np.diff(index)) > 2*average:
        raise ValueError('There is a gait cycle detected with 2 times gait duration'
                         + 'than the averaged gait cycle')
        
    return new_index2
        
index_l_hs = remove_outlayer(np.where(np.diff(index_l) == 1)[0])
index_r_hs = remove_outlayer(np.where(np.diff(index_r) == 1)[0])

averaged_gait_duration_l = np.mean(np.diff(index_l_hs))
averaged_gait_duration_r = np.mean(np.diff(index_r_hs))

average_l_ang = np.zeros((101, 18))
average_r_ang = np.zeros((101, 18))

average_l_mom = np.zeros((101, 6))
average_r_mom = np.zeros((101, 6))

average_l_grf = np.zeros((101, 6))
average_r_grf = np.zeros((101, 6))

for m in range(len(index_l_hs)-1):
    
    time_average = np.linspace(0, (index_l_hs[m+1]-index_l_hs[m])*0.01, 101, endpoint=True)
    time_real_l = np.linspace(0, (index_l_hs[m+1]-index_l_hs[m])*0.01, index_l_hs[m+1]-index_l_hs[m])
    for l in range(18):
        if l == 0:
            average_l_ang[:, l] += np.interp(time_average, time_real_l, 
                         (motion1[index_l_hs[m]:index_l_hs[m+1], l] - motion1[index_l_hs[m], l]))
        else:
            average_l_ang[:, l] += np.interp(time_average, time_real_l, 
                         (motion1[index_l_hs[m]:index_l_hs[m+1], l]))
            
        if l < 6:
            average_l_mom[:, l] += np.interp(time_average, time_real_l, moment1[index_l_hs[m]:index_l_hs[m+1], l])
            average_l_grf[:, l] += np.interp(time_average, time_real_l, grf1[index_l_hs[m]:index_l_hs[m+1], l])
            
    
for n in range(len(index_r_hs)-1):
    
    time_average = np.linspace(0, (index_r_hs[n+1]-index_r_hs[n])*0.01, 101, endpoint=True)
    time_real_r = np.linspace(0, (index_r_hs[n+1]-index_r_hs[n])*0.01, index_r_hs[n+1]-index_r_hs[n])
    for l in range(18):
        if l == 0:
            average_r_ang[:, l] += np.interp(time_average, time_real_r, 
                         (motion1[index_r_hs[n]:index_r_hs[n+1], l] - motion1[index_r_hs[n], l]))
        else:
            average_r_ang[:, l] += np.interp(time_average, time_real_r, motion1[index_r_hs[n]:index_r_hs[n+1], l])
        
        if l < 6:
            average_r_mom[:, l] += np.interp(time_average, time_real_r, moment1[index_r_hs[n]:index_r_hs[n+1], l])
            average_r_grf[:, l] += np.interp(time_average, time_real_r, grf1[index_r_hs[n]:index_r_hs[n+1], l])
    
    
average_l_ang = average_l_ang/(len(index_l_hs)-1)
average_l_mom = average_l_mom/(len(index_l_hs)-1)
average_l_grf = average_l_grf/(len(index_l_hs)-1)

average_r_ang = average_r_ang/(len(index_r_hs)-1)
average_r_mom = average_r_mom/(len(index_r_hs)-1)
average_r_grf = average_r_grf/(len(index_r_hs)-1)

fig1 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 1 Motion')
ax1 = plt.subplot(3, 2, 1)
plt.ylabel('Left Hip (deg)')
ax2 = plt.subplot(3, 2, 2)
plt.ylabel('Right Hip (deg)')
ax3 = plt.subplot(3, 2, 3)
plt.ylabel('Left Knee (deg)')
plt.xlabel('Time (sec)')
ax4 = plt.subplot(3, 2, 4)
plt.ylabel('Right Knee (deg)')
ax5 = plt.subplot(3, 2, 5)
plt.ylabel('Left Ankle (deg)')
ax6 = plt.subplot(3, 2, 6)
plt.ylabel('Right Ankle (deg)')
plt.xlabel('Time (sec)')

ax1.plot( average_l_ang[:, 3]*180/pi, '.')
ax3.plot( average_l_ang[:, 4]*180/pi, '.')
ax5.plot( average_l_ang[:, 5]*180/pi, '.')

ax2.plot( average_r_ang[:, 6]*180/pi, '.')
ax4.plot( average_r_ang[:, 7]*180/pi, '.')
ax6.plot( average_r_ang[:, 8]*180/pi, '.')


fig4 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 1 Motion')
ax1 = plt.subplot(3, 2, 1)
plt.ylabel('Left x (m)')
ax2 = plt.subplot(3, 2, 2)
plt.ylabel('Right x (m)')
ax3 = plt.subplot(3, 2, 3)
plt.ylabel('Left y (m)')
plt.xlabel('Time (sec)')
ax4 = plt.subplot(3, 2, 4)
plt.ylabel('Right y (m)')
ax5 = plt.subplot(3, 2, 5)
plt.ylabel('Left rotation (deg)')
ax6 = plt.subplot(3, 2, 6)
plt.ylabel('Right rotation (deg)')
plt.xlabel('Time (sec)')

ax1.plot( average_l_ang[:, 0], '.')
ax3.plot( average_l_ang[:, 1], '.')
ax5.plot( average_l_ang[:, 2]*180/pi, '.')

ax2.plot( average_r_ang[:, 0], '.')
ax4.plot( average_r_ang[:, 1], '.')
ax6.plot( average_r_ang[:, 2]*180/pi, '.')


fig2 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 1 GRF')
ax21 = plt.subplot(3, 2, 1)
plt.ylabel('Left Fx (deg)')
ax22 = plt.subplot(3, 2, 2)
plt.ylabel('Right Fx (deg)')
ax23 = plt.subplot(3, 2, 3)
plt.ylabel('Left Fy (deg)')
plt.xlabel('Time (sec)')
ax24 = plt.subplot(3, 2, 4)
plt.ylabel('Right Fy (deg)')
ax25 = plt.subplot(3, 2, 5)
plt.ylabel('Left Mx (deg)')
ax26 = plt.subplot(3, 2, 6)
plt.ylabel('Right Mx (deg)')
plt.xlabel('Time (sec)')

ax21.plot( average_l_grf[:, 0], '.')
ax23.plot( average_l_grf[:, 1], '.')
ax25.plot( average_l_grf[:, 2], '.')

ax22.plot( average_r_grf[:, 0], '.')
ax24.plot( average_r_grf[:, 1], '.')
ax26.plot( average_r_grf[:, 2], '.')


fig3 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 2 Moment')
ax31 = plt.subplot(3, 2, 1)
plt.ylabel('Left Hip (Nm)')
ax32 = plt.subplot(3, 2, 2)
plt.ylabel('Right Hip (Nm)')
ax33 = plt.subplot(3, 2, 3)
plt.ylabel('Left Knee (Nm)')
plt.xlabel('Time (sec)')
ax34 = plt.subplot(3, 2, 4)
plt.ylabel('Right Knee (Nm)')
ax35 = plt.subplot(3, 2, 5)
plt.ylabel('Left Ankle (Nm)')
ax36 = plt.subplot(3, 2, 6)
plt.ylabel('Right Ankle (Nm)')
plt.xlabel('Time (sec)')


ax31.plot( average_l_mom[:, 0], '.')
ax33.plot( average_l_mom[:, 1], '.')
ax35.plot( average_l_mom[:, 2], '.')

ax32.plot( average_r_mom[:, 3], '.')
ax34.plot( average_r_mom[:, 4], '.')
ax36.plot( average_r_mom[:, 5], '.')


index_l = np.zeros(101)
index_r = np.zeros(101)

ho_l = 42
to_l = 62
kv_l = 73

ho_r = 42
to_r = 62
kv_r = 73

index_l[0:ho_l] = 1
index_l[ho_l:to_l] = 2
index_l[to_l:kv_l] = 3
index_l[kv_l:] = 4

index_r[0:ho_r] = 1
index_r[ho_r:to_r] = 2
index_r[to_r:kv_r] = 3
index_r[kv_r:] = 4

if j < 4:
    store_path_ave = 'averaged_normal_gaits_1_2dmodel/F'+str(j)+'_S'+str(s)
else:
    store_path_ave = 'averaged_normal_gaits_1_2dmodel/M'+str(j-4)+'_S'+str(s)

if not os.path.exists(store_path_ave):
    os.makedirs(store_path_ave)

with open(store_path_ave + '/averaged_joint_angles_l.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        for p in range(18):
            StringP += str(average_l_ang[o, p])
            StringP += ' '
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_joint_moments_l.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        for p in range(6):
            StringP += str(average_l_mom[o, p])
            StringP += ' '
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_grf_l.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        for p in range(6):
            StringP += str(average_l_grf[o, p])
            StringP += ' '
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_index_l.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        StringP += str(index_l[o])
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_gait_duration_l.txt', 'w') as Outfile:
    StringP = ''
    StringP += str(averaged_gait_duration_l*0.01)
    StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_joint_angles_r.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        for p in range(18):
            StringP += str(average_r_ang[o, p])
            StringP += ' '
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_joint_moments_r.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        for p in range(6):
            StringP += str(average_r_mom[o, p])
            StringP += ' '
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_grf_r.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        for p in range(6):
            StringP += str(average_r_grf[o, p])
            StringP += ' '
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_index_r.txt', 'w') as Outfile:
    StringP = ''
    for o in range(101):
        StringP += str(index_r[o])
        StringP += '\n'
    Outfile.write(StringP)
    
with open(store_path_ave + '/averaged_gait_duration_r.txt', 'w') as Outfile:
    StringP = ''
    StringP += str(averaged_gait_duration_r*0.01)
    StringP += '\n'
    Outfile.write(StringP)