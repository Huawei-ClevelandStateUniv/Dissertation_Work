#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May  3 14:56:40 2018

This code is to check derivative of gait 2d mode with perturbation

@author: huawei
"""

import numpy as np
from gait2dpi_model_CL_simulation_SelectedJointsControl import gait2dpi_model
from scipy.sparse import find
import yaml

num_states = 18
num_cons = 18

start_node = 1000
num_nodes = 51
interval = 0.02
duration = (num_nodes-1)/interval

num_normal = 50

index_open = np.array([0, 2, 3, 4, 5])
index_control = np.array([1])

num_open = len(index_open)
num_control = len(index_control)

# control information
num_phase = 4
num_par = num_control*num_phase*2


weight_mt = 1e-2 
weight_ms = 1e-3
weight_ts = 1e-5
weight_tm = 1e-7
weight_tp = 1e-1
Scaling = 1

# information of the subject
gender = 'F'
S = 1
T = 1

mass = 60

processed_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/processed_data/'
          + gender + str(S) + '_S' + str(T) + '/')

selected_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/selected_data/'
          + gender + str(S) + '_S' + str(T) + '/')

normal_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/averaged_normal_gaits_1_2dmodel/'
          + gender + str(S) + '_S' + str(T) + '/')

# load gait experimental data
motion_meas = np.loadtxt(selected_path + 'Motion_2dmodel_30000_40000_50HZ.txt')[start_node:start_node+num_nodes, :]
moment_meas = np.loadtxt(selected_path + 'Moment_2dmodel_30000_40000_50HZ.txt')[start_node:start_node+num_nodes, :]
vs_meas = -np.loadtxt(selected_path + 'Belt_30000_40000_50HZ.txt') [start_node:start_node+num_nodes, :]

# load processed average gait data
normal_al = np.loadtxt(normal_path + 'averaged_joint_angles_l.txt')[:, 3:6]
normal_ar = np.loadtxt(normal_path + 'averaged_joint_angles_r.txt')[:, 6:]
normal_a = np.hstack((normal_al, normal_ar))

normal_ml = np.loadtxt(normal_path + 'averaged_joint_moments_l.txt')[:, :3]
normal_mr = np.loadtxt(normal_path + 'averaged_joint_moments_r.txt')[:, 3:]
normal_m = np.hstack((normal_ml, normal_mr))

# load pre-detected phases
index_normal = np.loadtxt(selected_path + 'phase_index_30000_40000_50Hz.txt')[start_node:start_node+num_nodes, :]
index2_imped = np.loadtxt(selected_path + 'four_phases_index.txt')[start_node:start_node+num_nodes, :]

# load model parameters
with open(processed_path + 'Model_Parameters_2.yml', 'r') as f:
    constants_dict = yaml.load(f)
constants_dict['kc'] = 1e5
constants_dict['ce'] = 0.001

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

Prob=gait2dpi_model(x_c_meas_vec, x_normal_ref, vs_meas, num_nodes, num_states, index_open,
                    index_control, num_normal, num_par, interval,
                    weight_mt, weight_ms, weight_ts, weight_tm, weight_tp, 
                    index_normal, index2_imped, constants_dict, scaling = Scaling)

print('Checking derivatives...')
            
# generate random motion state x, xd and controls u
x0 = x_c_meas_vec + 0.1*np.random.random(num_nodes*num_states)
u0 = mom_c_meas_vec
uo0 = mom_c_norm_vec
p0 = np.random.random(num_par)

vs = vs_meas
x0 = np.hstack((x0, u0, uo0, p0))

Jac0 = Prob.jacobian(x0)
Grad0 = Prob.gradient(x0)

Row, Col = Prob.jacobianstructure()

Unit_x_meas = np.eye(num_nodes*num_states + num_open*num_nodes + num_normal*num_control + num_par)

Jac_PD = np.zeros(((num_nodes - 1)*num_states, num_nodes*num_states 
                   + num_open*num_nodes + num_normal*num_control + num_par))
Jac_0 = np.zeros(((num_nodes - 1)*num_states, num_nodes*num_states 
                  + num_open*num_nodes + num_normal*num_control + num_par))
Ind_PD = np.zeros(((num_nodes - 1)*num_states, num_nodes*num_states 
                   + num_open*num_nodes + num_normal*num_control + num_par))

Grad_PD = np.zeros(num_nodes*num_states + num_open*num_nodes + num_normal*num_control + num_par)

eplison = 0.00001

for k in range(num_nodes*num_states + num_open*num_nodes + num_normal*num_control + num_par):
    ConU = Prob.constraints(x0 + eplison*Unit_x_meas[:, k])
    ConD = Prob.constraints(x0 - eplison*Unit_x_meas[:, k])
    
    ObjU = Prob.objective(x0 + eplison*Unit_x_meas[:, k])
    ObjD = Prob.objective(x0 - eplison*Unit_x_meas[:, k])
    
    Jac_PD[:, k] = (ConU - ConD)/(2*eplison)
    
    Grad_PD[k] = (ObjU - ObjD)/(2*eplison)
    
row_PD, col_PD, NA_PD = find(Jac_PD)    

RowInt = np.int_(Row)
ColInt = np.int_(Col)

Jac_0[RowInt, ColInt] = Jac0
     
diff_Jac = abs(Jac_0 - Jac_PD)

diff_Grad = abs(Grad0 - Grad_PD)

Ind_PD[RowInt, ColInt] = 1
      
Maximum_diff_Jac = max(diff_Jac[RowInt, ColInt])

RelDiff_Jac = np.zeros_like(Jac_0)

for k in range(len(RowInt)):
    if np.abs(Jac_0[RowInt[k], ColInt[k]]) > 1e-5:
        RelDiff_Jac[RowInt[k], ColInt[k]] = diff_Jac[RowInt[k], ColInt[k]]/abs(Jac_0[RowInt[k], ColInt[k]])
    
Maximum_diff_Grad = max(diff_Grad)

Index1 = np.where(Maximum_diff_Jac == diff_Jac)
Index2 = np.where(RelDiff_Jac.max() == RelDiff_Jac)



print('Maximum error in Jac is: %s  at Row %s Col %s  Relative Error: %s' 
      % (Maximum_diff_Jac, Index1[0], Index1[1], RelDiff_Jac[Index1[0][0], Index1[1][0]]))


print('Maximum relative error in Jac is: %s  at Row %s Col %s  Error: %s' 
      % (RelDiff_Jac.max(), Index2[0], Index2[1], diff_Jac[Index2[0][0], Index2[1][0]]))


print('Maximum number in Grad is: %s' % (Maximum_diff_Grad))