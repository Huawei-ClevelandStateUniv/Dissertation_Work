#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 25 17:59:21 2017

Revised on Wed May 02 12:13:00 2018


@author: huawei
"""

import ipopt
import numpy as np
from gait2dpi_model_CL_simulation_SelectedJointsControl import gait2dpi_model
from Pink_noise import generate_pink_noise
from numpy import sqrt, mean, square
import yaml
import time
import os
import matplotlib.pyplot as plt

# identification trails
id_sta = 5
id_end = 10

# identification settings
start_node = 1500
num_nodes = 501
interval = 0.02
duration = (num_nodes-1)*interval
num_states = 18
num_cons = num_states
num_joints = 7

num_normal = 50

index_open = np.array([0, 3, 4, 5])
index_control = np.array([1, 2])

num_open = len(index_open)
num_control = len(index_control)

# control information
num_phase = 4
num_par = num_control*num_phase*2


weight_mt = 30 
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

simulation_data_path = ('../Simulation_Study_Data/Simulation_2dData_Wmt100Wms0.005Wts1e-05Wtm1e-07Wtp0.1/'
          + gender + str(S) + '_S' + str(T) + '_N501_JC2_Phase4GoodInit2/')

processed_path = ('../walking_data/processed_data/'
          + gender + str(S) + '_S' + str(T) + '/')

selected_path = ('../walking_data/selected_data/'
          + gender + str(S) + '_S' + str(T) + '/')

normal_path = ('../walking_data/averaged_normal_gaits_1_2dmodel/'
          + gender + str(S) + '_S' + str(T) + '/')

# load gait experimental data
motion_sim = np.loadtxt(simulation_data_path + 'Simulated_motion.txt')
moment_sim = np.loadtxt(simulation_data_path + 'Simulated_torques.txt')
grf_meas = np.loadtxt(selected_path + 'Grf_2dmodel_30000_40000_50HZ.txt')[start_node:start_node+num_nodes, :]
vs_meas = np.loadtxt(selected_path + 'Belt_30000_40000_50HZ.txt') [start_node:start_node+num_nodes, :]
imped_sim = np.loadtxt(simulation_data_path + 'Simulated_imped.txt')
moment_pero = np.loadtxt(simulation_data_path + 'Simulated_torques_pero.txt')
moment_impe = np.loadtxt(simulation_data_path + 'Simulated_torques_impe.txt')

# load processed average gait data

normal_al = np.loadtxt(normal_path + 'averaged_joint_angles_l.txt')[:, 3:6]
normal_ar = np.loadtxt(normal_path + 'averaged_joint_angles_r.txt')[:, 6:]
normal_a = np.hstack((normal_al, normal_ar))

normal_ml = np.loadtxt(normal_path + 'averaged_joint_moments_l.txt')[:, :3]
normal_mr = np.loadtxt(normal_path + 'averaged_joint_moments_r.txt')[:, 3:]
normal_m = np.hstack((normal_ml, normal_mr))

# load pre-detected phases
index_normal = np.loadtxt(selected_path + 'phase_index_30000_40000_50Hz.txt')[start_node:start_node+num_nodes, :]
if num_phase == 2:
    index2_imped = np.loadtxt(selected_path + 'phase_2_index_30000_40000_50Hz.txt')[start_node:start_node+num_nodes, :]
    index2_imped[:, 1] = 3 - index2_imped[:, 1]
    impedance = np.array([1.75, 0.0125, 0.1, 0.015, 3.75, 0.011, 0.5, 0.015])*mass  # ankle and knee joints phase 2
elif num_phase == 4:
    index2_imped = np.loadtxt(selected_path + 'four_phases_index.txt')[start_node:start_node+num_nodes, :]
    impedance = np.array([7.5, 0.005, 1.0, 0.02, 0.1, 0.01, 0.1, 0.02, 
                              2.5, 0.012, 5, 0.01, 0.5, 0.017, 0.5, 0.014])*mass   # ankle and knee joints phase 4

# check whether the ankle joint is defined correctly, change to right if not.
    
if np.mean(normal_a[:, 2]) > np.pi/2:
    normal_a[:, 2] -= np.pi
elif np.mean(normal_a[:, 2]) < -np.pi/2:
    normal_a[:, 2] += np.pi
    
if np.mean(normal_a[:, 5]) > np.pi/2:
    normal_a[:, 5] -= np.pi
elif np.mean(normal_a[:, 5]) < -np.pi/2:
    normal_a[:, 5] += np.pi
    
# setting forward walking speeds

motion_sim[:, 0] = motion_sim[:, 0] + np.linspace(0, num_nodes*interval, num_nodes)*(0.8 + 0.4*T)
vs_meas = -vs_meas + 0.8 + 0.4*T

#fig1 = plt.figure(figsize=(12,8))
#ax1 = fig1.add_subplot(3, 2, 1)
#ax2 = fig1.add_subplot(3, 2, 2)
#ax3 = fig1.add_subplot(3, 2, 3)
#ax4 = fig1.add_subplot(3, 2, 4)
#ax5 = fig1.add_subplot(3, 2, 5)
#ax6 = fig1.add_subplot(3, 2, 6)
#
#color = ['k.', 'b.', 'g.', 'r.']
#for i in range(len(motion_sim[:, 0])):
#    ax1.plot(i, motion_sim[i, 3], color[int(index2_imped[i, 0]-1)])
#    ax1.plot(i, motion_sim[i, 4], color[int(index2_imped[i, 0]-1)])
#    ax1.plot(i, motion_sim[i, 5], color[int(index2_imped[i, 0]-1)])
#    
#    ax2.plot(i, motion_sim[i, 6], color[int(index2_imped[i, 1]-1)])
#    ax2.plot(i, motion_sim[i, 7], color[int(index2_imped[i, 1]-1)])
#    ax2.plot(i, motion_sim[i, 8], color[int(index2_imped[i, 1]-1)])
#    
#    ax3.plot(i, grf_meas[i, 0], color[int(index2_imped[i, 0]-1)])
#    ax3.plot(i, grf_meas[i, 1], color[int(index2_imped[i, 0]-1)])
#    
#    ax4.plot(i, grf_meas[i, 3], color[int(index2_imped[i, 1]-1)])
#    ax4.plot(i, grf_meas[i, 4], color[int(index2_imped[i, 1]-1)])
#    
#    ax5.plot(i, moment_sim[i, 0], color[int(index2_imped[i, 0]-1)])
#    ax5.plot(i, moment_sim[i, 1], color[int(index2_imped[i, 0]-1)])
#    ax5.plot(i, moment_sim[i, 2], color[int(index2_imped[i, 0]-1)])
#    
#    ax6.plot(i, moment_sim[i, 3], color[int(index2_imped[i, 1]-1)])
#    ax6.plot(i, moment_sim[i, 4], color[int(index2_imped[i, 1]-1)])
#    ax6.plot(i, moment_sim[i, 5], color[int(index2_imped[i, 1]-1)])


# define the optimization problem
x_c_meas_vec = motion_sim.T.flatten('F')
mom_c_meas_vec = moment_sim[:, index_open].T.flatten('F')
mom_c_norm_vec = moment_pero[:num_normal :].T.flatten('F')

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
    
#plt.figure()
#plt.plot(x_normal_ref[:, [0, 1]])
#plt.plot(motion_sim[:, [4 ,5]], '--')
#
#plt.figure()
#plt.plot(x_normal_ref[:, [2, 3]])
#plt.plot(motion_sim[:, [13 ,14]], '--')
           
# impedance = np.array([2.5, 0.012, 5, 0.01, 0.5, 0.017, 0.5, 0.014])*mass  # ankle joint phase 4
# impedance = np.array([3.75, 0.011, 0.5, 0.015])*mass  # ankle joint phase 2

i_x = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int)
i_y = i_x + 1
i_torso = i_x + 2
i_RHip = i_x + 3
i_RKnee = i_x + 4
i_RAnkle = i_x + 5
i_LHip = i_x + 6
i_LKnee = i_x + 7
i_LAnkle = i_x + 8
                                     
i_xv = i_x + 9
i_yv = i_x + 10
i_torsov = i_x + 11
i_RHipv = i_x + 12
i_RKneev = i_x + 13
i_RAnklev = i_x + 14
i_LHipv = i_x + 15
i_LKneev = i_x + 16
i_LAnklev = i_x + 17

lb_x = np.zeros((num_states*num_nodes))

lb_x[i_x] = motion_sim[:, 0] - 0.0
lb_x[i_y] = 0.7
lb_x[i_torso] = -0.3

lb_x[i_RHip] = -1.0
lb_x[i_RKnee] = -1.5
lb_x[i_RAnkle] = -1.0
    
lb_x[i_LHip] = -1.0
lb_x[i_LKnee] = -1.5
lb_x[i_LAnkle] = -1.0
    
lb_x[i_xv] = 0.2
lb_x[i_yv] = -0.5
lb_x[i_torsov] = -1.0

lb_x[i_RHipv] = -4.0
lb_x[i_RKneev] = -7.0
lb_x[i_RAnklev] = -7.0
    
lb_x[i_LHipv] = -4.0
lb_x[i_LKneev] = -7.0
lb_x[i_LAnklev] = -7.0

lb_u = np.zeros(num_nodes*num_open) - 1500
lb_uo = np.zeros(num_normal*num_control) - 1500
lb_p = np.array([0.5, 0, 0.5, 0, 0, 0, 0, 0, 
             0.5, 0, 0.5, 0, 0, 0, 0, 0])*mass
                  
ub_x = np.zeros((num_states*num_nodes))

ub_x[i_x] = motion_sim[:, 0] + 0.5
ub_x[i_y] = 1.1
ub_x[i_torso] = 0.3

ub_x[i_RHip] = 1.0
ub_x[i_RKnee] = 0.1
ub_x[i_RAnkle] = 1.0
    
ub_x[i_LHip] = 1.0
ub_x[i_LKnee] = 0.1
ub_x[i_LAnkle] = 1.0
    
ub_x[i_xv] = 1.4
ub_x[i_yv] = 0.5
ub_x[i_torsov] = 1.0

ub_x[i_RHipv] = 4.0
ub_x[i_RKneev] = 7.0
ub_x[i_RAnklev] = 7.0
    
ub_x[i_LHipv] = 4.0
ub_x[i_LKneev] = 7.0
ub_x[i_LAnklev] = 7.0
               
ub_u = np.zeros(num_nodes*num_open) + 1500
ub_uo = np.zeros(num_normal*num_control) + 1500
ub_p = np.array([10, 0.5, 10, 0.5, 10, 0.5, 10, 0.5, 
             10, 0.5, 10, 0.5, 10, 0.5, 10, 0.5])*mass
    
lb = np.hstack((lb_x, lb_u, lb_uo, lb_p))
ub = np.hstack((ub_x, ub_u, ub_uo, ub_p))

cl = np.zeros((num_cons*(num_nodes -1)))
cu = np.zeros((num_cons*(num_nodes -1)))

# load model parameters
with open(processed_path + 'Model_Parameters_2.yml', 'r') as f:
    constants_dict = yaml.load(f)
constants_dict['kc'] = 1e5
constants_dict['ce'] = 0.001
constants_dict['fyd'] = -0.06 # actual foot size when generating simulation data
constants_dict['hxd'] = -0.051 # actual foot size when generating simulation data
constants_dict['txd'] = 0.137 # actual foot size when generating simulation data

#    constants_dict['ma']  = (0.9 + 0.2*rand_list[0])*constants_dict['ma']  # TrunkMass
#    constants_dict['ia']  = (0.9 + 0.2*rand_list[1])*constants_dict['ia']  # TrunkInertia
#    constants_dict['ya']  = (0.9 + 0.2*rand_list[2])*constants_dict['ya']  # TrunkCMy
#    constants_dict['mb']  = (0.9 + 0.2*rand_list[3])*constants_dict['mb']  # ThighMass
#    constants_dict['ib']  = (0.9 + 0.2*rand_list[4])*constants_dict['ib']  # ThighInertia
#    constants_dict['yb']  = (0.9 + 0.2*rand_list[5])*constants_dict['yb']  # ThighCMy
#    constants_dict['mc']  = (0.9 + 0.2*rand_list[6])*constants_dict['mc']  # ShankMass
#    constants_dict['ic']  = (0.9 + 0.2*rand_list[7])*constants_dict['ic']  # ShankInertia
#    constants_dict['yc']  = (0.9 + 0.2*rand_list[8])*constants_dict['yc']  # ShankInertia
#    constants_dict['md']  = (0.9 + 0.2*rand_list[9])*constants_dict['md']  # ShankInertia
#    constants_dict['idd']  = (0.9 + 0.2*rand_list[10])*constants_dict['idd']  # ShankInertia
#    constants_dict['xd']  = (0.9 + 0.2*rand_list[11])*constants_dict['xd']  # ShankInertia
#    constants_dict['yd']  = (0.9 + 0.2*rand_list[12])*constants_dict['yd']  # ShankInertia

nlp = ipopt.problem(
            n=num_states*num_nodes + num_nodes*num_open + num_normal*num_control + num_par,
            m=num_cons*(num_nodes -1),
            problem_obj = gait2dpi_model(x_c_meas_vec, x_normal_ref, vs_meas, num_nodes, num_states, index_open,
                    index_control, num_normal, num_par, interval,
                    weight_mt, weight_ms, weight_ts, weight_tm, weight_tp, 
                    index_normal, index2_imped, constants_dict, scaling = Scaling),
            lb=lb,
            ub=ub,
            cl=cl,
            cu=cu
            )

nlp.addOption(b'linear_solver', b'MA86')
nlp.addOption(b'hessian_approximation', b'limited-memory')
nlp.addOption(b'max_iter', 15000)
nlp.addOption(b'tol', 1e-4)
nlp.addOption(b'acceptable_tol', 1e-3)
nlp.addOption(b'max_cpu_time', 2e+5)

store_path = ('../Simulation_Study_Data/Simulation_Id_ReasonableRange/')

if not os.path.exists(store_path):
    os.makedirs(store_path)


for ll in range(id_sta, id_end):

    x_init = x_c_meas_vec + 0.001*np.random.random(num_nodes*num_states)
    u_init = mom_c_meas_vec
    uo_init = mom_c_norm_vec
    p_init = lb_p + np.random.random(len(lb_p))*(ub_p - lb_p)

    x0 =  np.hstack((x_init, u_init, uo_init, p_init))
    
    sta_time = time.time()
    x, info = nlp.solve(x0)
    end_time = time.time()
    
    Vec_RHip = x_c_meas_vec[i_RHip]-x[i_RHip]
    Vec_LHip = x_c_meas_vec[i_LHip]-x[i_LHip]
    Vec_RKnee = x_c_meas_vec[i_RKnee]-x[i_RKnee]
    Vec_LKnee = x_c_meas_vec[i_LKnee]-x[i_LKnee]
    Vec_RAnkle = x_c_meas_vec[i_RAnkle]-x[i_RAnkle]
    Vec_LAnkle = x_c_meas_vec[i_LAnkle]-x[i_LAnkle]
    
    Vec_rms_hip = mean(square(np.hstack((Vec_RHip, Vec_LHip))))
    Vec_rms_knee = mean(square(np.hstack((Vec_RKnee, Vec_LKnee))))
    Vec_rms_ankle = mean(square(np.hstack((Vec_RAnkle, Vec_LAnkle))))
    
    RMS = sqrt((Vec_rms_hip + Vec_rms_knee + Vec_rms_ankle)/3.0)
    
    Sta = info['status']
    R_cst = info['g']
    
    FitName = store_path + 'TrajectoryResult_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(ll) + '.txt'
    with open(FitName,'w') as Outfile:
    
        for m in range(0,num_nodes):
            StringP = ""
            for n in range(0,num_states):
                StringP += str(x[m*num_states + n])
                StringP += " "
            StringP += "\n"
            Outfile.write(StringP)
            
    RcstName = store_path + 'ResiduleConstraints_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(ll) + '.txt'
    with open(RcstName,'w') as Outfile:
        StringP = ""
        for m in range(0,len(cl)):
            StringP += str(R_cst[m])
            StringP += "\n"
        Outfile.write(StringP)
    
    
    ContrName = store_path + 'Torques_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(ll) + '.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for i in range(0,num_nodes):
            for j in range(num_open):
                StringP += str(x[num_nodes*num_states + i*num_open + j])
                StringP += " "
            StringP += "\n"
        Outfile.write(StringP)
        
    ContrName = store_path + 'Normal_Torques_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(ll) + '.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for i in range(0,num_normal):
            for j in range(num_control):
                StringP += str(x[num_nodes*num_states + num_nodes*num_open + i*num_control + j])
                StringP += " "
            StringP += "\n"
        Outfile.write(StringP)
        
    ImpedName = store_path + 'Impedance_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(ll) + '.txt'
    with open(ImpedName,'w') as Outfile:
        StringP = ""
        for i in range(0,num_par):
            StringP += str(x[num_nodes*num_states + num_nodes*num_open + num_normal*num_control + i])
            StringP += "\n"
        Outfile.write(StringP)
    
    RMS_Sta_Name = store_path + 'RMS_Sta_Scaling'+str(Scaling)+'HStiff_SmoothGround_50HZ_' + str(ll) + '.txt'
    with open(RMS_Sta_Name,'w') as Outfile:
        StringP = ""
        StringP += str(RMS)
        StringP += "\n"
        StringP += str(Sta)
        StringP += "\n"
        StringP += str(end_time - sta_time)
        StringP += "\n"
        Outfile.write(StringP)
