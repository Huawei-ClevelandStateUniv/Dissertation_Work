#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  1 14:39:18 2019

This code is to generate the index indication vector for gait cycles

@author: huawei
"""

import numpy as np

def index_indication_vector_generator(phase1, phase2, phase3, phase4, num_nodes):
    """
    Inputs:
        phase1: index vector for phase 1 (heel strike)
        phase2: index vector for phase 2 (heel off)
        phase3: index vector for phase 3 (toe off)
        phase4: index vector for phase 4 (knee velocity change)
        num_nodes: total number of experiment nodes of this period of walking data
        
    Output:
        index_vector: an index indication vector for this period of walking data
    
    
    """

    min_size = np.min(len(phase1), len(phase2), len(phase3), len(phase4))
    
    index_mat = np.hstack((phase1[:min_size], phase2[:min_size], phase3[:min_size], phase4[:min_size]))
    
    min_index = np.argmin(index_mat[0, :])
    
    index_vector = np.zeros(num_nodes)
    
    for i in range(min_size):
        
        phase_sign = min_index
        
        for j in range(3):
            
            if phase_sign + j < 3:
                start_index = index_mat[i, phase_sign + j]
                end_index = index_mat[i, phase_sign + j + 1]
                
            elif phase_sign + j == 3:
                start_index = index_mat[i, phase_sign + j]
                end_index = index_mat[i, 0]
                
            else:
                start_index = index_mat[i, phase_sign + j - 4]
                end_index = index_mat[i, phase_sign + j - 3]
                
            index_vector[start_index:end_index] = phase_sign + j + 1
            
                
        start_index = index_mat[i, phase_sign -1]
        end_index = index_mat[i+1, phase_sign]
        
        index_vector[start_index:end_index] = phase_sign
            
    return index_vector
