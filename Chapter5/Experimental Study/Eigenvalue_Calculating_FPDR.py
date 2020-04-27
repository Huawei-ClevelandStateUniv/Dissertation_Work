#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 18:54:43 2018

This code is the calculating eigenvalues of identified SPD controller

@author: huawei
"""

import numpy as np
from StandingModel_FPD_Eps_Noise import Model

num_subj = 8
num_tria = 2
num_repeat = 3

num_nodes = 5001
num_states = 4
num_par = 10
interval = 0.01
Scaling = 100.0

x_exp2 = np.zeros(num_states*num_nodes)
acc_exp2 = np.zeros(num_nodes)
noise_exp2 = np.zeros((num_nodes, int(num_states/2)))

parameter = np.loadtxt('Data/Model Parameter_2DoF/Para_Winter_Subj0'+str(num_subj)+'.txt')

Dictionary = ('FPD/Analysis/Subj0'+ str(num_subj) +'/Pert'+str(num_tria) 
              + '_GFL_FPD_Eps' + str(num_repeat) + '_Noise025/')

problem_obj=Model(x_exp2, acc_exp2, num_repeat, num_nodes, num_states,
                  num_par, interval, parameter, noise_exp2, scaling =Scaling,
                  integration_method='midpoint')

x0 = np.zeros(num_states)
xdot0 = np.zeros(num_states)

a = np.zeros(1)

p = np.loadtxt(Dictionary + 'Sort_Con.txt')

# p[num_par-2:] = p[num_par-2:]*3.1416/180.0

noise = np.zeros(int(num_states/2))
with open(Dictionary + 'Eigenvalue_Sort.txt','w') as Outfile1:
    StringP = ""
    for k in range(10):
        f, dfdx, dfdxdot, dfdp = problem_obj.dynamic_fun(x0, xdot0, p[k,:], a, noise)
        
        Mat_A = -np.dot(dfdx, np.linalg.inv(dfdxdot))
        
        Eng, Vec = np.linalg.eig(Mat_A)
        
        for m in range(0,num_states):
            StringP += str(Eng[m])
            StringP += " "
        StringP += "\n"
    Outfile1.write(StringP)
    
with open(Dictionary + 'Eigenvalue_RealImag.txt','w') as Outfile:
    StringP = ""
    for k in range(10):
        f, dfdx, dfdxdot, dfdp = problem_obj.dynamic_fun(x0, xdot0, p[k,:], a, noise)
        
        Mat_A = -np.dot(dfdx, np.linalg.inv(dfdxdot))
        
        Eng, Vec = np.linalg.eig(Mat_A)
        
        for m in range(0,num_states):
            StringP += str(Eng.real[m])
            StringP += " "
            StringP += str(Eng.imag[m])
            StringP += " "
        StringP += "\n"
    Outfile.write(StringP)



