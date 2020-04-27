# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 18:39:41 2018

@author: Huawei
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

gender = 'F'

for S in range(4):
    for T in range(3):
#S = 0
#T = 1
        start_node = 30000
        end_node = 40000
        
        num_nodes = int((end_node - start_node)/2)
        
        select_nodes = np.linspace(start_node, end_node, num_nodes, endpoint=True, dtype=int)
        
        processed_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/processed_data/'
                          + gender + str(S) + '_S' + str(T) + '/')
        
        store_path = ('/home/huawei/Research/Research_Work/Ipopt_work/Walking_Balance/walking_data/selected_data/'
                          + gender + str(S) + '_S' + str(T) + '/')
        
        grf_select = np.loadtxt(processed_path + 'grf.txt')[select_nodes]
        
        
        b, a = signal.butter(2, 2.0*8/(50), 'low', analog=False)
        LFy = signal.filtfilt(b, a, grf_select[:, 1])
        RFy = signal.filtfilt(b, a, grf_select[:, 7])
        
        signL = np.zeros(num_nodes)
        signR = np.zeros(num_nodes)
        
        for indL in range(len(LFy)):
            if LFy[indL] > 100:
                signL[indL] = 1
                
        for indR in range(len(RFy)):
            if RFy[indR] > 100:
                signR[indR] = 1
            
        DsignL = np.diff(signL)
        DsignR = np.diff(signR)
        
        Lhs = (np.where(DsignL==1)[0]).astype(int)
        
        Rhs = (np.where(DsignR==1)[0]).astype(int)
        
        phase_ind = np.zeros((num_nodes, 2))
        
        
        
        for k in range(len(Lhs)-1):
            phase_ind[Lhs[k]:Lhs[k+1], 0] = np.linspace(0, 49, Lhs[k+1] - Lhs[k], endpoint=True)
            
        for j in range(len(Rhs)-1):
            phase_ind[Rhs[j]:Rhs[j+1], 1] = np.linspace(0, 49, Rhs[j+1] - Rhs[j], endpoint=True)
                
                
        with open(store_path + 'grf_3d_30000_40000_50Hz.txt', 'w') as outfile:
            StringP = ''
            for ii in range(len(grf_select[:, 0])):
                for jj in range(len(grf_select[0, :])):
                    StringP += str(grf_select[ii, jj])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path + 'phase_index_30000_40000_50Hz.txt', 'w') as outfile:
            StringP = ''
            for ii in range(len(phase_ind[:, 0])):
                for jj in range(len(phase_ind[0, :])):
                    StringP += str(phase_ind[ii, jj])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
                
                
                
                