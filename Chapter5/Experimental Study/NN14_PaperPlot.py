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
Ed = 10

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

num_par = 2 + num_states*2 + 2 + 2

num_subj = 6

Fit_Info = np.zeros((num_subj, 2, 3))

g = 9.8

for sj in range(num_subj):

    subj = 3 + sj

    for tr in range(2):
        
        store_path = 'NN14/Analysis_Paper/Subj0'+str(subj)+'Pert'+str(tr+1)+'_GFL_NN14/'
        
        Fit_Info[sj, tr, :] = np.loadtxt(store_path + 'Best_RMS_R.txt')
        
        
with open('NN14/Analysis_Paper/Lowest_RMS.txt', 'w') as outfile:
    StringP = ''
    for i in range(num_subj):
        for j in range(2):
            StringP += str(Fit_Info[i, j, 0])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open('NN14/Analysis_Paper/Highest_R1.txt', 'w') as outfile:
    StringP = ''
    for i in range(num_subj):
        for j in range(2):
            StringP += str(Fit_Info[i, j, 1])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)
    
with open('NN14/Analysis_Paper/Highest_R2.txt', 'w') as outfile:
    StringP = ''
    for i in range(num_subj):
        for j in range(2):
            StringP += str(Fit_Info[i, j, 2])
            StringP += ' '
        StringP += '\n'
    outfile.write(StringP)