#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  1 15:58:34 2017

@author: huawei
"""

import ipopt
import numpy as np
from StandingModel_GoodWorth_Opt2 import Model
from numpy import sqrt, mean, square
import time
import os


St = 0
Ed = 10
for nn in range(10):

    num_nodes = 4501
    interval = 0.02
    duration = (num_nodes-1)*interval
    
    index_select = np.linspace(0, (num_nodes-1)*2, num_nodes, endpoint = True, dtype=int)
    
    start_nodes = 8000
    
    subj = 4
    trial = 1
    
    num_states = 4
    num_cons = 4
    Scaling = 50.0
    integration_method = 'midpoint'
    max_delay_time = 0.18
    half_state = int(num_states/2)
    
    num_par = 2 + num_states*2 + 2 + 2
    
    Dictionary = 'GoodWorth3_Id_dur'+str(duration)+'Sca'+str(Scaling)+'/Simulation'+str(nn)+'/'
    if not os.path.exists(Dictionary):
        os.makedirs(Dictionary)
        
    Accel_String1 = ('Data/2DoF Data Simplify/Subj0'+str(subj)+'/SwayL000'+str(trial+1)+'.txt')
    X_String1 = ('ForwardSimulation_GoodWorth/DoublePendulum_N10001Noise_'+str(nn)+'.txt')
    Para_string = ('Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt') 
    
    accel_meas = -np.loadtxt(Accel_String1, usecols=(3))[start_nodes:start_nodes+num_nodes]
    x_meas = np.loadtxt(X_String1)[index_select, :]
    parameter = np.loadtxt(Para_string)
    
    Con_GoodWorth = np.array([152.76, 19.09, 891.07, 286.42, 151.48, 19.09, 251.41,
                              175.03, 50.70, 28.00, 0.09, 0.08, 0, 0])
    
    itheta_a = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int)
    itheta_h = np.linspace(0, num_states*num_nodes, num_nodes, endpoint=False, dtype=int) + 1
    
    lb_x = np.zeros((num_states*num_nodes)) - 2.0
    lb_x[itheta_a] = np.zeros((num_nodes)) - 1.0
    lb_x[itheta_h] = np.zeros((num_nodes)) - 1.0
    lb_x[:num_states] = x_meas[0, :]
                  
    ub_x = np.zeros((num_states*num_nodes)) + 2.0
    ub_x[itheta_a] = np.zeros((num_nodes)) + 1.0
    ub_x[itheta_h] = np.zeros((num_nodes)) + 1.0
    ub_x[:num_states] = x_meas[0, :]
    
    cl = np.zeros((num_cons*(num_nodes -1)))
    cu = np.zeros((num_cons*(num_nodes -1)))
    
    lb_c = np.zeros(num_par)
    lb_c[2+2*num_states:2+2*num_states+2] = 1e-8
    lb_c[2+2*num_states+2:2+2*num_states+4] = 0.0
    
    ub_c = 2*Con_GoodWorth
    ub_c[:-4] = ub_c[:-4]/Scaling
    
    lb1 = np.hstack((lb_x, lb_c))
    ub1 = np.hstack((ub_x, ub_c))
    
    x_meas_vec = x_meas.T.flatten('F')
    
    nlp1 = ipopt.problem(
                n=num_states*num_nodes + num_par,
                m=num_cons*(num_nodes -1),
                problem_obj=Model(x_meas_vec, accel_meas, num_nodes, num_states, num_par,
                                  max_delay_time, interval, parameter, scaling = Scaling,
                                  integration_method = integration_method),
                lb=lb1,
                ub=ub1,
                cl=cl,
                cu=cu
                )
    
    nlp1.addOption(b'linear_solver', b'MA86')
    nlp1.addOption(b'max_iter', 10000)
    nlp1.addOption(b'hessian_approximation', b'limited-memory')
    nlp1.addOption(b'tol', 1e-4)
    nlp1.addOption(b'acceptable_tol', 1e-3)
    nlp1.addOption(b'max_cpu_time', 2e+4)
    
    for i in range(St, Ed):
        
        np.random.seed()
        
        par0 = (2*Con_GoodWorth*np.random.random(num_par))
        par0[:-4] = par0[:-4]/Scaling
        
        if par0[-4] < 1e-8:
            par0[-4] = 1e-8
        if par0[-3] < 1e-8:
            par0[-3] = 1e-8
        
        x0 =  np.hstack((x_meas_vec, par0))
         
        start_time = time.time()
        x, info = nlp1.solve(x0)
        TimeCost = time.time() - start_time
        
        Vec_a = x_meas_vec[itheta_a]-x[itheta_a]
        Vec_h = x_meas_vec[itheta_h]-x[itheta_h]
        
            
        Vec_RMS = np.hstack((Vec_a, Vec_h))
        RMS = sqrt(mean(mean(square(Vec_RMS))))
        Sta = info['status']
        
        FitName = Dictionary + 'TrajectoryResult_'+ str(i) +'.txt'     
        with open(FitName,'w') as Outfile:
            StringP = ""
            for m in range(0,num_nodes):
                for n in range(0,num_states):
                    StringP += str(x[m*num_states + n])
                    StringP += " "
                StringP += "\n"
            Outfile.write(StringP)
            
        ResName = Dictionary + 'ConstraintsResidule_'+ str(i) +'.txt'     
        with open(ResName,'w') as Outfile:
            StringP = ""
            for m in range(0,num_nodes-1):
                for n in range(0,num_states):
                    StringP += str(info['g'][m*num_states + n])
                    StringP += " "
                StringP += "\n"
            Outfile.write(StringP)
                
        ContrName = Dictionary + 'Controller_'+ str(i) + '.txt'
        with open(ContrName,'w') as Outfile:
            StringP = ""
            for q in range(0,num_par):
                StringP += str(x[num_nodes*num_states + q])
                StringP += "\n"
            Outfile.write(StringP)
        
        RMS_Sta_Name = Dictionary + 'RMS_Sta_'+ str(i) + '.txt'
        with open(RMS_Sta_Name,'w') as Outfile:
            StringP = ""
            StringP += str(RMS)
            StringP += "\n"
            StringP += str(Sta)
            StringP += "\n"
            StringP += str(TimeCost)
            StringP += "\n"
            Outfile.write(StringP)