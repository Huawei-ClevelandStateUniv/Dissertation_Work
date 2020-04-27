#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 00:04:02 2019

@author: huawei
"""

import numpy as np
    
    
def normal_torque(uo, index_open, num_control, num_normal, index_control_motion):
    
    tor = np.zeros(num_control)
    
    i_normalT_local = np.zeros((num_control, num_normal))
    
    for j in range(num_control):
        i_normalT_local[j, :] = np.linspace(0, num_control*num_normal, endpoint=False, dtype=int) + j
        
    i_normalT_local = i_normalT_local.astype(int)

    for i in range(num_control):
        
        if index_control_motion[i] < 6:
            
            if index_open[0] == 49:
                
                tor[i] = uo[i_normalT_local[i, int(index_open[0])]]

            else:
                
                tor[i] = ((-index_open[0] + 1 + int(index_open[0]))*uo[i_normalT_local[i, int(index_open[0])]] + 
                     (index_open[0] - int(index_open[0]))*uo[i_normalT_local[i, int(index_open[0])+1]])
                
        else:
            
            if index_open[1] == 49:
                
                tor[i] = uo[i, i_normalT_local[i, int(index_open[1])]]
                
            else:
                tor[i] = ((-index_open[1] +1 + int(index_open[1]))*uo[i_normalT_local[i, int(index_open[1])]] + 
                     (index_open[1] - int(index_open[1]))*uo[i_normalT_local[i, int(index_open[1])+1]])

    return tor