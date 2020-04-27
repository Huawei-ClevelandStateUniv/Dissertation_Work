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
        Lto = (np.where(DsignL==-1)[0]).astype(int)
        Rhs = (np.where(DsignR==1)[0]).astype(int)
        Rto = (np.where(DsignR==-1)[0]).astype(int)
        
        Lgait = np.min([len(Lhs), len(Lto)])
        Rgait = np.min([len(Rhs), len(Rto)])
        
        phase_50_ind = np.zeros((num_nodes, 2))
        phase_2_ind = np.zeros((num_nodes, 2))
        
        
        if Lto[0] < Lhs[0]:
            for k in range(Lgait-1):
                phase_50_ind[Lto[k]:Lhs[k], 0] = np.linspace(0, 24, Lhs[k] - Lto[k], endpoint=True)
                phase_50_ind[Lhs[k]:Lto[k+1], 0] = np.linspace(25, 49, Lto[k+1] - Lhs[k], endpoint=True)
                
                phase_2_ind[Lto[k]:Lhs[k], 0] = 2
                phase_2_ind[Lhs[k]:Lto[k+1], 0] = 1
                
        else:
            for k in range(Lgait-1):
                phase_50_ind[Lhs[k]:Lto[k], 0] = np.linspace(0, 24, Lto[k] - Lhs[k], endpoint=True)
                phase_50_ind[Lto[k]:Lhs[k+1], 0] = np.linspace(25, 49, Lhs[k+1] - Lto[k], endpoint=True)
                
                phase_2_ind[Lhs[k]:Lto[k], 0] = 2
                phase_2_ind[Lto[k]:Lhs[k+1], 0] = 1
                
                
        if Rto[0] < Rhs[0]:
            for k in range(Rgait-1):
                phase_50_ind[Rto[k]:Rhs[k], 1] = np.linspace(0, 24, Rhs[k] - Rto[k], endpoint=True)
                phase_50_ind[Rhs[k]:Rto[k+1], 1] = np.linspace(25, 49, Rto[k+1] - Rhs[k], endpoint=True)
                
                phase_2_ind[Rto[k]:Rhs[k], 1] = 2
                phase_2_ind[Rhs[k]:Rto[k+1], 1] = 1
                
        else:
            for k in range(Rgait-1):
                phase_50_ind[Rhs[k]:Rto[k], 1] = np.linspace(0, 24, Rto[k] - Rhs[k], endpoint=True)
                phase_50_ind[Rto[k]:Rhs[k+1], 1] = np.linspace(25, 49, Rhs[k+1] - Rto[k], endpoint=True)
                
                phase_2_ind[Rhs[k]:Rto[k], 1] = 2
                phase_2_ind[Rto[k]:Rhs[k+1], 1] = 1
                
                
                
        with open(store_path + 'grf_3d_30000_40000_50Hz.txt', 'w') as outfile:
            StringP = ''
            for ii in range(len(grf_select[:, 0])):
                for jj in range(len(grf_select[0, :])):
                    StringP += str(grf_select[ii, jj])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path + 'phase_50_index_30000_40000_50Hz.txt', 'w') as outfile:
            StringP = ''
            for ii in range(len(phase_50_ind[:, 0])):
                for jj in range(len(phase_50_ind[0, :])):
                    StringP += str(phase_50_ind[ii, jj])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path + 'phase_2_index_30000_40000_50Hz.txt', 'w') as outfile:
            StringP = ''
            for ii in range(len(phase_2_ind[:, 0])):
                for jj in range(len(phase_2_ind[0, :])):
                    StringP += str(phase_2_ind[ii, jj])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
                
                
                
                