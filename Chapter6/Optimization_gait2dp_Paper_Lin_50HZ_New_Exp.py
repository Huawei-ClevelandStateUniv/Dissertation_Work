#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 25 17:59:21 2017

Revised on Wed May 02 12:13:00 2018


@author: huawei
"""

import ipopt
import numpy as np
from Gait2dpiModel_Paper_Lin import gait2dpi_model
from numpy import sqrt, mean, square
import time
import yaml
import os


S = 4
T = 2
gender = 'M'

start_nodes = 3000
num_nodes = 501
duration = 10.0

interval = duration/(num_nodes - 1)

num_states = 22
num_cons = 22

num_con_close = 2 + 4 + 0
num_con_ankle = (num_nodes-1)*2

Scaling = 1.0
lamda = 0.99
repeat1 = 0
repeat2 = 10

store_path = 'result_id_new_exp_3000/'+gender+str(S)+'_S'+str(T)+'/'
if not os.path.exists(store_path):
    os.makedirs(store_path)
    
processed_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/processed_data/'
                  + gender + str(S) + '_S' + str(T) + '/')
    
selected_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/selected_data/'
                  + gender + str(S) + '_S' + str(T) + '/')

with open(processed_path + 'Model_Parameters_2.yml', 'r') as f:
    constants_dict = yaml.load(f)
constants_dict['kc'] = 1e5
constants_dict['ce'] = 0.005
constants_dict['fyd'] = -0.09
constants_dict['fyg'] = -0.09

vs_meas_full = np.loadtxt(selected_path + 'Belt_30000_40000_50HZ.txt')

vs_meas = -vs_meas_full[start_nodes:start_nodes+num_nodes, :] + 0.8

initL = np.loadtxt(selected_path + 'InitialDataL_30000_40000_50HZ.txt')[start_nodes:start_nodes+num_nodes, :]
initR = np.loadtxt(selected_path + 'InitialDataR_30000_40000_50HZ.txt')[start_nodes:start_nodes+num_nodes, :]

coe_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/swing_stance_path/')
coe_path_subj = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/swing_stance_path/'
                  + gender + str(S) + '_S' + str(T) + '/')

swingx_coe = np.loadtxt(coe_path_subj + 'Coe_Swingx_normalized5_std.txt')
swingy_coe = np.loadtxt(coe_path_subj + 'Coe_Swingy_normalized5_std.txt')

if gender == 'F':
    x_meas = np.loadtxt(selected_path + 'Motion_30000_40000_50HZ.txt')[start_nodes:start_nodes+num_nodes, :]
    torque_open = np.loadtxt('open_result_new_param2/'+gender+str(S)+'_S'+str(T)+'/'
                         +'Torques_Scaling1.0_Lamda0.99nodes501HStiff_SmoothGround_50HZ_1.txt')[:num_nodes, :]
    swingy_endp = np.loadtxt(coe_path + 'swingy_endpoints.txt')[S, T]
    stancex_endp = np.loadtxt(coe_path + 'stancex_endpoints.txt')[S, T]
elif gender == 'M':
    x_meas = np.loadtxt(selected_path + 'Motion_30000_40000_50HZ.txt')[start_nodes:start_nodes+num_nodes, :]
    torque_open = np.loadtxt('open_result_new_param2/'+gender+str(S)+'_S'+str(T)+'/'
                         +'Torques_Scaling1.0_Lamda0.99nodes501_50HZ1_SmoothContact.txt')[:num_nodes, :]
    swingy_endp = np.loadtxt(coe_path + 'swingy_endpoints.txt')[S+4, T]
    stancex_endp = np.loadtxt(coe_path + 'stancex_endpoints.txt')[S+4, T]

x_meas[:, 0] = x_meas[:, 0] + np.linspace(0, duration, num_nodes)*0.8


num_con_Lopen = int(np.sum(initL[:num_nodes, 0]))
num_con_Ropen = int(np.sum(initR[:num_nodes, 0]))

swingL_index = np.where(initL[:num_nodes, 0]==0)[0]
swingR_index = np.where(initR[:num_nodes, 0]==0)[0]

stanceL_index = np.where(initL[:num_nodes, 0]==1)[0]
stanceR_index = np.where(initR[:num_nodes, 0]==1)[0]

i_x = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int)
i_y = i_x + 1
i_torso = i_x + 2
i_LHip = i_x + 3
i_LKnee = i_x + 4
i_LAnkle = i_x + 5
i_RHip = i_x + 6
i_RKnee = i_x + 7
i_RAnkle = i_x + 8

i_xv = i_x + 9
i_yv = i_x + 10
i_torsov = i_x + 11
i_LHipv = i_x + 12
i_LKneev = i_x + 13
i_LAnklev = i_x + 14
i_RHipv = i_x + 15
i_RKneev = i_x + 16
i_RAnklev = i_x + 17

i_HipRef = i_x + 18
i_KneeRef = i_x + 19

i_HipVelRef = i_x + 20
i_KneeVelRef = i_x + 21

lb_x = np.zeros((num_states*num_nodes))

lb_x[i_x] = x_meas[:, 0] - 0.5
lb_x[i_y] = 0.6
lb_x[i_torso] = -0.3

lb_x[i_LHip] = -1.0
lb_x[i_LKnee] = -1.5
lb_x[i_LAnkle] = -1.0

lb_x[i_RHip] = -1.0
lb_x[i_RKnee] = -1.5
lb_x[i_RAnkle] = -1.0

lb_x[i_xv] = 0.2
lb_x[i_yv] = -0.5
lb_x[i_torsov] = -1.0

lb_x[i_LHipv] = -4.0
lb_x[i_LKneev] = -7.0
lb_x[i_LAnklev] = -7.0

lb_x[i_RHipv] = -4.0
lb_x[i_RKneev] = -7.0
lb_x[i_RAnklev] = -7.0

lb_x[i_HipRef] = -1.0
lb_x[i_KneeRef] = -1.5

lb_x[i_HipVelRef] = -4.0
lb_x[i_KneeVelRef] = -7.0
          
lb_con_close = np.zeros((num_con_close)) -5
lb_con_close[-4:-2] = 500
lb_con_close[-2:] = 100

lb_con_Lopen = np.zeros((num_con_Lopen*2)) -1500
lb_con_Ropen = np.zeros((num_con_Ropen*2)) -1500
lb_con_ankle = np.zeros((num_con_ankle)) -1500
#              
ub_x = np.zeros((num_states*num_nodes))

ub_x[i_x] = x_meas[:, 0] + 0.5
ub_x[i_y] = 1.0
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

ub_x[i_HipRef] = 1.0
ub_x[i_KneeRef] = 0.1

ub_x[i_HipVelRef] = 4.0
ub_x[i_KneeVelRef] = 7.0
               
ub_con_close = np.zeros((num_con_close)) + 5
ub_con_close[-4:-2] = 5000
ub_con_close[-2:] = 1000

ub_con_Lopen = np.zeros((num_con_Lopen*2)) + 1500
ub_con_Ropen = np.zeros((num_con_Ropen*2)) + 1500
ub_con_ankle = np.zeros((num_con_ankle)) + 1500

lb = np.hstack((lb_x, lb_con_close, lb_con_Lopen, lb_con_Ropen, lb_con_ankle))
ub = np.hstack((ub_x, ub_con_close, ub_con_Lopen, ub_con_Ropen, ub_con_ankle))

cl = np.zeros(num_cons*(num_nodes -1))
cu = np.zeros(num_cons*(num_nodes -1))
      
x_meas_full = np.hstack((x_meas[:num_nodes, :18], np.zeros((num_nodes, 4))))

x_meas_full[swingL_index, -4:] = (x_meas_full[:, np.array([3, 4, 12 ,13])][swingL_index, :] 
                                    + 0.1*np.random.random((len(swingL_index), 4)))
x_meas_full[swingR_index, -4:] = (x_meas_full[:, np.array([6, 7, 15, 16])][swingR_index, :] 
                                    + 0.1*np.random.random((len(swingR_index), 4)))

x_c_meas_vec = x_meas_full.T.flatten('F')
vs_c_meas = vs_meas[:num_nodes,:]

nlp = ipopt.problem(
            n=num_states*num_nodes + num_con_close + num_con_Lopen*2 + num_con_Ropen*2 +num_con_ankle,
            m=num_cons*(num_nodes -1),
            problem_obj=gait2dpi_model(x_c_meas_vec, vs_c_meas, initL, initR, 
                                       num_nodes, num_states, num_con_close, 
                                       num_con_Lopen, num_con_Ropen, num_con_ankle,
                                       swingx_coe[:, 0], swingy_coe[:, 0], swingy_endp,
                                       stancex_endp, constants_dict,
                                       interval, lamda, scaling =Scaling),
            lb=lb,
            ub=ub,
            cl=cl,
            cu=cu
            )
               
nlp.addOption(b'linear_solver', b'MA86')
nlp.addOption(b'max_iter', 15000)
nlp.addOption(b'hessian_approximation', b'limited-memory')
nlp.addOption(b'tol', 1e-4)
nlp.addOption(b'acceptable_tol', 1e-3)
nlp.addOption(b'max_cpu_time', 2e+5)

x_init = x_c_meas_vec

for p in range(repeat1, repeat2):

    con_close = 1 - 2*np.random.random(num_con_close)
    con_close[-4:-2] = 500 + 4500*np.random.random(2)
    con_close[-2:] = 100 + 900*np.random.random(2)
    #con_close[-1] = 2*np.random.random(1)
    
    con_Lopen = torque_open[stanceL_index, :2].T.flatten('F')
    con_Ropen = torque_open[stanceR_index, 3:5].T.flatten('F')
    con_ankle = torque_open[:num_nodes-1, [2, 5]].T.flatten('F')
    
    x0 =  np.hstack((x_init, con_close, con_Lopen, con_Ropen, con_ankle))
    start = time.time()
    x, info = nlp.solve(x0)
    end = time.time()
    
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
    
    FitName = store_path + 'TrajectoryResult_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(FitName,'w') as Outfile:
        for m in range(0,num_nodes):
            StringP = ""
            for n in range(0,num_states):
                StringP += str(x[m*num_states + n])
                StringP += " "
            StringP += "\n"
            Outfile.write(StringP)
            
    RcstName = store_path + 'ResiduleConstraints_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(RcstName,'w') as Outfile:
        StringP = ""
        for m in range(0,len(cl)):
            StringP += str(R_cst[m])
            StringP += "\n"
        Outfile.write(StringP)
    
    ContrName = store_path + 'Controller_close_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for m in range(0,num_con_close):
            StringP += str(x[num_nodes*num_states + m])
            StringP += " "
        StringP += "\n"
        Outfile.write(StringP)
        
    ContrName = store_path + 'Controller_openL_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for m in range(0,num_con_Lopen):
            StringP += str(x[num_nodes*num_states + num_con_close + 2*m])
            StringP += " "
            StringP += str(x[num_nodes*num_states + num_con_close + 2*m+1])
            StringP += "\n"
        Outfile.write(StringP)
        
    ContrName = store_path + 'Controller_openR_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for m in range(0,num_con_Ropen):
            StringP += str(x[num_nodes*num_states + num_con_close + num_con_Lopen*2 + 2*m])
            StringP += " "
            StringP += str(x[num_nodes*num_states + num_con_close + num_con_Lopen*2 + 2*m+1])
            StringP += "\n"
        Outfile.write(StringP)
        
    ContrName = store_path + 'Controller_ankle_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for m in range(0,int(num_con_ankle/2)):
            StringP += str(x[num_nodes*num_states + num_con_close + num_con_Lopen*2 + num_con_Ropen*2 + 2*m])
            StringP += " "
            StringP += str(x[num_nodes*num_states + num_con_close + num_con_Lopen*2 + num_con_Ropen*2 + 2*m+1])
            StringP += "\n"
        Outfile.write(StringP)
    
    RMS_Sta_Name = store_path + 'RMS_Sta_S'+str(Scaling)+'_L'+str(lamda)+'_N'+str(num_nodes)+'_FixPD'+str(p)+'.txt'
    with open(RMS_Sta_Name,'w') as Outfile:
        StringP = ""
        StringP += str(RMS)
        StringP += "\n"
        StringP += str(Sta)
        StringP += "\n"
        StringP += str(end-start)
        StringP += "\n"
        Outfile.write(StringP)