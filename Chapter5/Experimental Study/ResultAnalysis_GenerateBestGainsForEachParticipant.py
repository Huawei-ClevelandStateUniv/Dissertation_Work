# -*- coding: utf-8 -*-
"""
Created on Thu Apr 13 09:46:29 2017

Result analysis of RepIniCon Trails


@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from IPython.core.pylabtools import figsize
# import os
from scipy import signal
from scipy import stats

Data_RMS_All = np.zeros((6, 2))

for Subj in range(3, 9):
    for Trail in range(1, 3):
#        Subj = 3
#        Trail = 1
        
        RepTimes = 10
        Scaling = 100.0
        Show_Num = 10
        
        start_nodes = 8000
        end_nodes = 13001
        start_time = start_nodes/50.0
        end_time = end_nodes/50.0
        
        num_states = 4
        num_repeat = 2
        num_nodes = 5001
        delay_nodes = 0

        
        folderE = ('Data/2DoF Data Simplify/Subj0'+str(Subj)+'/')
        
        X_String = folderE + 'JointsGFL000' + str(Trail+1) + '.txt'
        Acc_String = folderE + 'SwayL000' + str(Trail+1) + '.txt'
      
        x_meas = np.loadtxt(X_String, usecols=(1, 2, 3, 4))*3.1416/180.0
        acc_meas = np.loadtxt(Acc_String, usecols=(3))

        Data_RMS_All[Subj-3, Trail-1] = np.sqrt(sum(x_meas[start_nodes:end_nodes, 0]**2 + x_meas[start_nodes:end_nodes, 1]**2)/num_nodes)
        
                
with open('Data/2DoF Data Simplify/Data_RMS.txt','w') as Outfile:
    StringP = ""
    for i in range(len(Data_RMS_All[:, 0])):
        for j in range(len(Data_RMS_All[0, :])):
            StringP += str(Data_RMS_All[i, j])
            StringP += " "
        StringP += "\n"
    Outfile.write(StringP)
