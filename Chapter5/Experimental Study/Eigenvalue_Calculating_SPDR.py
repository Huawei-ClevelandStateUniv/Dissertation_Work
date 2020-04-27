#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 18:54:43 2018

This code is the calculating eigenvalues of identified SPD controller

@author: huawei
"""

import numpy as np
from StandingModel_SPD_Eps import Model

num_subj = 5
num_tria = 1

num_repeat = 4
num_unopt = 500
num_nodes = 5001
num_states = 4
num_par = 6
interval = 0.01
Scaling = 1.0

x_exp2 = np.zeros(num_states*num_nodes)
acc_exp2 = np.zeros(num_nodes)

parameter = np.loadtxt('Model Parameter_2DoF/Para_Winter_Subj0'+str(num_subj)+'.txt')

Dictionary = ('Analysis_150_250/Subj0'+ str(num_subj) +'/Pert'+str(num_tria) 
              + '_GFL_SPD50_new_acc_sign_Eps' + str(num_repeat) + '_Noise_Random/')

problem_obj=Model(x_exp2, acc_exp2, num_repeat, num_unopt, num_nodes, num_states,
                  num_par, interval, parameter, scaling =Scaling,
                  integration_method='midpoint')

x0 = np.zeros(num_states)
xdot0 = np.zeros(num_states)

a = np.zeros(1)

p = np.loadtxt(Dictionary + 'Best_Con.txt')

p[num_par-2:] = p[num_par-2:]*3.1416/180.0

f, dfdx, dfdxdot, dfdp = problem_obj.dynamic_fun(x0, xdot0, p, a)

Mat_A = -np.dot(dfdx, np.linalg.inv(dfdxdot))

Eng, Vec = np.linalg.eig(Mat_A)

with open(Dictionary + 'Eigenvalue.txt','w') as Outfile:
    StringP = ""
    for m in range(0,num_states):
        StringP += str(Eng[m])
        StringP += " "
        StringP += "\n"
    Outfile.write(StringP)



