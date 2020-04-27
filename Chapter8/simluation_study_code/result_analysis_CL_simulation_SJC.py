#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 24 10:01:11 2019

create video of identified open torque trajectories.

@author: huawei
"""
import sys
sys.path.append('../')

import numpy as np
import matplotlib.pyplot as plt
from annimation_open import animate_pendulum
from simulate import map_values_to_autolev_symbols
from gait2dpi_dGRF_LinContact import evaluate_autolev_rhs as autolev_rhs
from normal_torque_SJC import normal_torque
from Impedance_control_SJC_ResultAnalysis import impedance_control_SJC

import yaml

num_id = 8

# identification settings
start_node = 1000
num_nodes = 501
interval = 0.02
duration = (num_nodes-1)*interval
num_states = 18
num_cons = num_states

num_normal = 50

index_open = np.array([0, 3, 4, 5])
index_control = np.array([1, 2])

num_open = len(index_open)
num_control = len(index_control)

# control information
num_phase = 4
num_par = num_control*num_phase*2


weight_mt = 100
weight_ms = 5e-3
weight_ts = 1e-5
weight_tm = 1e-7
weight_tp = 1e-1
Scaling = 1

# information of the subject
gender = 'F'
S = 1
T = 1

mass = 60

processed_path = ('../walking_data/processed_data/'
          + gender + str(S) + '_S' + str(T) + '/')

selected_path = ('../walking_data/selected_data/'
          + gender + str(S) + '_S' + str(T) + '/')

normal_path = ('../walking_data/averaged_normal_gaits_1_2dmodel/'
          + gender + str(S) + '_S' + str(T) + '/')

# load gait experimental data
motion_meas = np.loadtxt(selected_path + 'Motion_2dmodel_30000_40000_50HZ.txt')[start_node:start_node+num_nodes, :]
moment_meas = np.loadtxt(selected_path + 'Moment_2dmodel_30000_40000_50HZ.txt')[start_node:start_node+num_nodes, :]
vs_meas = np.loadtxt(selected_path + 'Belt_30000_40000_50HZ.txt') [start_node:start_node+num_nodes, :]

# load processed average gait data
normal_al = np.loadtxt(normal_path + 'averaged_joint_angles_l.txt')[:, 3:6]
normal_ar = np.loadtxt(normal_path + 'averaged_joint_angles_r.txt')[:, 6:]
normal_a = np.hstack((normal_al, normal_ar))

normal_ml = np.loadtxt(normal_path + 'averaged_joint_moments_l.txt')[:, :3]
normal_mr = np.loadtxt(normal_path + 'averaged_joint_moments_r.txt')[:, 3:]
normal_m = np.hstack((normal_ml, normal_mr))

# load pre-detected phases
index_normal = np.loadtxt(selected_path + 'phase_index_30000_40000_50Hz.txt')[start_node:start_node+num_nodes, :]
index2_imped = np.loadtxt(selected_path + 'phase_2_index_30000_40000_50Hz.txt')[start_node:start_node+num_nodes, :]

# load model parameters
with open(processed_path + 'Model_Parameters_2.yml', 'r') as f:
    constants_dict = yaml.load(f)
constants_dict['kc'] = 1e5
constants_dict['ce'] = 0.001
constants_dict['fyd'] = -0.06
constants_dict['hxd'] = -0.051
constants_dict['txd'] = 0.137

# check whether the ankle joint is defined correctly, change to right if not.
if np.mean(motion_meas[:, 5]) > np.pi/2:
    motion_meas[:, 5] -= np.pi
elif np.mean(motion_meas[:, 5]) < -np.pi/2:
    motion_meas[:, 5] += np.pi
    
if np.mean(motion_meas[:, 8]) > np.pi/2:
    motion_meas[:, 8] -= np.pi
elif np.mean(motion_meas[:, 8]) < -np.pi/2:
    motion_meas[:, 8] += np.pi
    
if np.mean(normal_a[:, 2]) > np.pi/2:
    normal_a[:, 2] -= np.pi
elif np.mean(normal_a[:, 2]) < -np.pi/2:
    normal_a[:, 2] += np.pi
    
if np.mean(normal_a[:, 5]) > np.pi/2:
    normal_a[:, 5] -= np.pi
elif np.mean(normal_a[:, 5]) < -np.pi/2:
    normal_a[:, 5] += np.pi
    
# setting forward walking speeds

motion_meas[:, 0] = motion_meas[:, 0] - motion_meas[0, 0] # + np.linspace(0, num_nodes*interval, num_nodes)*(0.8 + 0.4*T)
vs_meas = -vs_meas # + 0.8 + 0.4*T

# define the optimization problem
x_c_meas_vec = motion_meas.T.flatten('F')
mom_c_meas_vec = moment_meas[:, index_open].T.flatten('F')
mom_c_norm_vec = normal_m[list(np.linspace(0, 100, num_normal, dtype=int)), :][:, index_control].T.flatten('F')

# calculate reference normal joint angles
normal_a_sel = normal_a[list(np.linspace(0, 100, num_normal, dtype=int)), :][:, index_control]
x_normal_ref = np.zeros((num_nodes, num_control*2))
for n in range(num_nodes):
    for a in range(num_control):
        if index_control[a] < 3:
            if index_normal[n, 0] == 49:
                x_normal_ref[n, a] = normal_a_sel[49, a]
            else:
                x_normal_ref[n, a] = ((-index_normal[n, 0] + int(index_normal[n, 0]) + 1)*normal_a_sel[int(index_normal[n, 0]), a] + 
                                      (index_normal[n, 0] - int(index_normal[n, 0]))*normal_a_sel[int(index_normal[n, 0] + 1), a])
        else:
            if index_normal[n, 1] == 49:
                x_normal_ref[n, a] = normal_a_sel[49, a]
            else:
                x_normal_ref[n, a] = ((-index_normal[n, 1] + int(index_normal[n, 1]) + 1)*normal_a_sel[int(index_normal[n, 1]), a] + 
                                      (index_normal[n, 1] - int(index_normal[n, 1]))*normal_a_sel[int(index_normal[n, 1] + 1), a])
                
for a in range(num_control):
    x_normal_ref[1:, num_control:] = (x_normal_ref[1:, :num_control] - x_normal_ref[:-1, :num_control])/interval


store_path = ('../Simulation_Study_Data/Simulation_2dData_Wmt'+str(weight_mt)+'Wms'+str(weight_ms)+'Wts'
              +str(weight_ts)+'Wtm'+str(weight_tm)+'Wtp'+str(weight_tp)+'/'
              +gender+str(S)+'_S'+str(T)+'_N'+str(num_nodes)+'_JC'+str(num_control)+'_Phase'+str(num_phase)+'GoodInit2/')
        
ntrial = num_id
rms = np.zeros(ntrial)
sta = np.zeros(ntrial)

for k in range(ntrial):
    
    #con_close[k, :] = np.loadtxt(store_path + 'Impedance_Scaling' + str(scaling) + '_WeightTorque' + str(weight_tor) + 'nodes' + str(num_nodes) + 'impedance_50HZ_'+str(k)+'.txt')
    rms[k] = np.loadtxt(store_path + 'RMS_Sta_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(k) + '.txt')[0]
    sta[k] = np.loadtxt(store_path + 'RMS_Sta_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(k) + '.txt')[1]
    
sta_ind = np.where(sta==0)[0]
rms_ind = np.argsort(rms[sta_ind])

ind = np.linspace(1, ntrial, ntrial, endpoint=True, dtype=np.int32)
width = 0.35
fig15 = plt.figure(figsize=(8, 3))
plt.bar(ind[:len(rms_ind)], rms[sta_ind[rms_ind]], width)
plt.ylabel('RMS (rad)')
plt.title('RMS distribution in 10 identifications')
plt.xlabel('Identification number')
fig15.savefig(store_path + 'RMS_rank.png')

time_plot = np.linspace(0, duration, num_nodes)

best_ind = sta_ind[rms_ind][0]
Result = np.loadtxt(store_path + 'TrajectoryResult_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(best_ind) + '.txt')
torque_open = np.loadtxt(store_path + 'Torques_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(best_ind) + '.txt')
torque_norm = np.loadtxt(store_path + 'Normal_Torques_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(best_ind) + '.txt')
impedance = np.loadtxt(store_path + 'Impedance_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(best_ind) + '.txt')

Result[:, 0] = Result[:, 0] - Result[0, 0] - np.linspace(0, num_nodes*interval, num_nodes)*(0.8 + T*0.4)
torque_norm_vec = torque_norm.T.flatten('F')

torque_all = np.zeros((num_nodes, 6))

torque_all[:, index_open] = torque_open   

torque_impe = np.zeros((num_nodes, num_control))
torque_pero = np.zeros((num_nodes, num_control))

index_control_motion = (index_control + 3).astype(int)

for n in range(num_nodes):
    torque_impe[n, :] = impedance_control_SJC(Result[n, :], x_normal_ref[n, :],
               impedance, index2_imped[n, :], num_control, num_par, index_control_motion)
    
    torque_pero[n, :] = normal_torque(torque_norm_vec, index_normal[n, :], num_control, num_normal, index_control_motion)

torque_all[:, index_control] = torque_impe + torque_pero

with open(store_path + 'Simulated_motion.txt', 'w') as outfile:
    StringP = ''
    for r in range(num_nodes):
        for c in range(18):
            StringP += str(Result[r, c])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open(store_path + 'Simulated_torques.txt', 'w') as outfile:
    StringP = ''
    for r in range(num_nodes):
        for c in range(6):
            StringP += str(torque_all[r, c])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open(store_path + 'Simulated_torques_impe.txt', 'w') as outfile:
    StringP = ''
    for r in range(num_nodes):
        for c in range(num_control):
            StringP += str(torque_impe[r, c])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open(store_path + 'Simulated_torques_pero.txt', 'w') as outfile:
    StringP = ''
    for r in range(num_nodes):
        for c in range(num_control):
            StringP += str(torque_pero[r, c])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)

with open(store_path + 'Simulated_vs.txt', 'w') as outfile:
    StringP = ''
    for r in range(num_nodes):
        for c in range(2):
            StringP += str(vs_meas[r, c])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open(store_path + 'Simulated_imped.txt', 'w') as outfile:
    StringP = ''
    for r in range(len(impedance)):
        StringP += str(impedance[r])
        StringP += ' '
    outfile.write(StringP)

plot_nodes = num_nodes

fig1 = plt.figure(figsize=(12, 3))
plt.plot(time_plot, motion_meas[:plot_nodes, 0], 'r-')
plt.plot(time_plot, Result[:plot_nodes, 0], 'b.', label='x translation (m)')

plt.plot(time_plot, motion_meas[:plot_nodes, 1], 'r-')
plt.plot(time_plot, Result[:plot_nodes, 1], 'g.', label='y translation (m)')

plt.plot(time_plot, motion_meas[:plot_nodes, 2], 'r-')
plt.plot(time_plot, Result[:plot_nodes, 2], 'm.', label='Trunk Rotation (rad)')
plt.xlabel('Time (s)', fontsize=14)
plt.ylabel('Trunk Motion', fontsize=14)
plt.legend(bbox_to_anchor=(1.003, 1), loc=2, borderaxespad=0., fontsize=14)
fig1.savefig(store_path + 'trunk_fit_trajectory.png')

fig2 = plt.figure(figsize=(16, 8))
ax1 = fig2.add_subplot(3, 2, 1)
plt.ylabel('Left Hip (rad)', fontsize=14)
ax2 = fig2.add_subplot(3, 2, 2)
plt.ylabel('Right Hip (rad)', fontsize=14)
ax3 = fig2.add_subplot(3, 2, 3)
plt.ylabel('Left Knee (rad)', fontsize=14)
ax4 = fig2.add_subplot(3, 2, 4)
plt.ylabel('Right Knee (rad)', fontsize=14)
ax5 = fig2.add_subplot(3, 2, 5)
plt.ylabel('Left Ankle (rad)', fontsize=14)
plt.xlabel('Time (s)', fontsize=14)
ax6 = fig2.add_subplot(3, 2, 6)
plt.ylabel('Right Ankle (rad)', fontsize=14)
plt.xlabel('Time (s)', fontsize=14)

ax1.plot(time_plot, motion_meas[:plot_nodes, 3], 'r.', linewidth=2.5, label='Experimental data')
ax1.plot(time_plot, Result[:plot_nodes, 3], '-', label='Identified Trajectory')

ax2.plot(time_plot, motion_meas[:plot_nodes, 6], 'r.', linewidth=2.5, label='Experimental data')
ax2.plot(time_plot, Result[:plot_nodes, 6], '-', label='Identified Trajectory')

ax3.plot(time_plot, motion_meas[:plot_nodes, 4], 'r.', linewidth=2.5, label='Experimental data')
ax3.plot(time_plot, Result[:plot_nodes, 4], '-', label='Identified Trajectory')

ax4.plot(time_plot, motion_meas[:plot_nodes, 7], 'r.', linewidth=2.5, label='Experimental data')
ax4.plot(time_plot, Result[:plot_nodes, 7], '-', label='Identified Trajectory')

ax5.plot(time_plot, motion_meas[:plot_nodes, 5], 'r.', linewidth=2.5, label='Experimental data')
ax5.plot(time_plot, Result[:plot_nodes, 5], '-', label='Identified Trajectory')

ax6.plot(time_plot, motion_meas[:plot_nodes, 8], 'r.', linewidth=2.5, label='Experimental data')
ax6.plot(time_plot, Result[:plot_nodes, 8], '-', label='Identified Trajectory')

plt.legend(bbox_to_anchor=(0.71, 0.25), loc=2, borderaxespad=0.)
fig2.savefig(store_path + 'joint_fit_trajectory.png')

fig3 = plt.figure(figsize=(16, 8))
ax11 = fig3.add_subplot(3, 2, 1)
plt.ylabel('Left Hip (Nm)', fontsize=14)
ax12 = fig3.add_subplot(3, 2, 2)
plt.ylabel('Right Hip (Nm)', fontsize=14)
ax13 = fig3.add_subplot(3, 2, 3)
plt.ylabel('Left Knee (Nm)', fontsize=14)
plt.legend()
ax14 = fig3.add_subplot(3, 2, 4)
plt.ylabel('Right Knee (Nm)', fontsize=14)
ax15 = fig3.add_subplot(3, 2, 5)
plt.ylabel('Left Ankle (Nm)', fontsize=14)
#plt.xlabel('Time (s)', fontsize=14)
ax16 = fig3.add_subplot(3, 2, 6)
plt.ylabel('Right Ankle (Nm)', fontsize=14)
#plt.xlabel('Time (s)', fontsize=14)

ax11.plot(time_plot[1:], moment_meas[1:num_nodes, 0], 'r.', linewidth=2.5, label='Inverse Dynamics')
ax11.plot(time_plot[1:], torque_all[1:, 0], '-', label='Indentified Torques')


ax12.plot(time_plot[1:], moment_meas[1:num_nodes, 3], 'r.', linewidth=2.5, label='Inverse Dynamics')
ax12.plot(time_plot[1:], torque_all[1:, 3], '-', label='Indentified Open Torques')
#ax12.plot(time_plot[1:], torque_pero[1:, 0], '.', label='Normal Average Torques')
#ax12.plot(time_plot[1:], torque_impe[1:, 0], '.', label='Impedance Control Torques')

ax13.plot(time_plot[1:], moment_meas[1:num_nodes, 1], 'r.', linewidth=2.5, label='Inverse Dynamics')
ax13.plot(time_plot[1:], torque_all[1:, 1], '-', label='Indentified Open Torques')
ax13.plot(time_plot[1:], torque_pero[1:, 0], '.', label='Normal Average Torques')
ax13.plot(time_plot[1:], torque_impe[1:, 0], '.', label='Impedance Control Torques')

ax14.plot(time_plot[1:], moment_meas[1:num_nodes, 4], 'r.', linewidth=2.5, label='Inverse Dynamics')
ax14.plot(time_plot[1:], torque_all[1:, 4], '-', label='Indentified Open Torques')
#ax14.plot(time_plot[1:], torque_nor[1:, 4], '.', label='Normal Average Torques')
#ax14.plot(time_plot[1:], torque_imp[1:, 4], '.', label='Impedance Control Torques')

ax15.plot(time_plot[1:], moment_meas[1:num_nodes, 2], 'r.', linewidth=2.5, label='Inverse Dynamics')
ax15.plot(time_plot[1:], torque_all[1:, 2], '-', label='Indentified Open Torques')
ax15.plot(time_plot[1:], torque_pero[1:, 1], '.', label='Normal Average Torques')
ax15.plot(time_plot[1:], torque_impe[1:, 1], '.', label='Impedance Control Torques')

ax16.plot(time_plot[1:], moment_meas[1:num_nodes, 5], 'r.', linewidth=2.5, label='Inverse Dynamics')
ax16.plot(time_plot[1:], torque_all[1:, 5], '-', label='Indentified Open Torques')
#ax16.plot(time_plot[1:], torque_nor[1:, 5], '.', label='Normal Average Torques')
#ax16.plot(time_plot[1:], torque_imp[1:, 5], '.', label='Impedance Control Torques')

plt.legend(bbox_to_anchor=(0.74, 1), loc=2, borderaxespad=0.)
fig3.savefig(store_path + 'torque_fit_trajectory.png')

if num_phase == 2:
    fig4 = plt.figure()
    ax1 = fig4.add_subplot(2,2,1)
    ax2 = fig4.add_subplot(2,2,2)
    ax3 = fig4.add_subplot(2,2,3)
    ax4 = fig4.add_subplot(2,2,4)
    
    xaxis = [1, 2]
    
    impedance = impedance/mass
    
    ax1.bar(xaxis, impedance[:2], align='center', alpha=0.3)
    ax3.bar(xaxis, impedance[2:4], align='center', alpha=0.3)
    
    ax2.bar(xaxis, impedance[4:6], align='center', alpha=0.3)
    ax4.bar(xaxis, impedance[6:8], align='center', alpha=0.3)
    
elif num_phase == 4:
    fig4 = plt.figure()
    ax1 = fig4.add_subplot(4,2,1)
    ax2 = fig4.add_subplot(4,2,2)
    ax3 = fig4.add_subplot(4,2,3)
    ax4 = fig4.add_subplot(4,2,4)
    ax5 = fig4.add_subplot(4,2,5)
    ax6 = fig4.add_subplot(4,2,6)
    ax7 = fig4.add_subplot(4,2,7)
    ax8 = fig4.add_subplot(4,2,8)
    
    xaxis = [1, 2]
    
    impedance = impedance/mass
    
    ax1.bar(xaxis, impedance[:2], align='center', alpha=0.3)
    ax3.bar(xaxis, impedance[2:4], align='center', alpha=0.3)
    ax5.bar(xaxis, impedance[4:6], align='center', alpha=0.3)
    ax7.bar(xaxis, impedance[6:8], align='center', alpha=0.3)
    
    ax2.bar(xaxis, impedance[8:10], align='center', alpha=0.3)
    ax4.bar(xaxis, impedance[10:12], align='center', alpha=0.3)
    ax6.bar(xaxis, impedance[12:14], align='center', alpha=0.3)
    ax8.bar(xaxis, impedance[14:16], align='center', alpha=0.3)

fig4.savefig(store_path + 'ForwardSimulation_ImpedanceGains.png')

constants_dict = map_values_to_autolev_symbols(constants_dict)
            
sticks_pred = np.zeros((num_nodes, 20))
qd = np.zeros(9)
qdd = np.zeros(9)
belts = np.zeros(num_nodes)
GRF = np.zeros((num_nodes, 6))

for o in range(num_nodes):
    _, _, _, _, GRF[o, :], _, _, sticks_pred[o, :] = \
                autolev_rhs(Result[o, :9], qd, qdd,
                            vs_meas[o, :], constants_dict)
    if o == 0:
        belts[o] = 0
    else:
        belts[o] = belts[o-1] + np.sum(vs_meas[o, :]-(0.8+0.4*T))/2*0.02

time_plot = np.linspace(0, num_nodes*interval, num_nodes)
anni_names = store_path + 'annimation.mp4'
fps_def = 12.5
animate_pendulum(time_plot, sticks_pred, belts, fps_def, filename=anni_names)


fig8 = plt.figure(figsize=(16, 8))
ax11 = fig8.add_subplot(3, 2, 1)
plt.ylabel('left Fx (N)', fontsize=14)
ax12 = fig8.add_subplot(3, 2, 2)
plt.ylabel('Right Fx (N)', fontsize=14)
ax13 = fig8.add_subplot(3, 2, 3)
plt.ylabel('Left Fy (N)', fontsize=14)
ax14 = fig8.add_subplot(3, 2, 4)
plt.ylabel('Right Fy (N)', fontsize=14)
ax15 = fig8.add_subplot(3, 2, 5)
plt.ylabel('Left Mx (Nm)', fontsize=14)
#plt.xlabel('Time (s)', fontsize=14)
ax16 = fig8.add_subplot(3, 2, 6)
plt.ylabel('Right Mx (Nm)', fontsize=14)
#plt.xlabel('Time (s)', fontsize=14)

ax11.plot(GRF[:, 0])
ax12.plot(GRF[:, 3])

ax13.plot(GRF[:, 1])
ax14.plot(GRF[:, 4])

ax15.plot(GRF[:, 2])
ax16.plot(GRF[:, 5])

fig8.savefig(store_path + 'Simulated_GRF.png')
