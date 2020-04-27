#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 00:04:13 2019

@author: huawei
"""

import numpy as np

def impedance_control_SJC(xs, xs_normal, p, index, num_control, num_par, index_control_motion):
        
        tor = np.zeros(num_control)
        
        par_joint = int(num_par/num_control)
        
        for i in range(num_control):
            if index_control_motion[i] < 6:
                tor[i] = (p[i*par_joint + int(index[0]-1)*2]*(xs_normal[i] 
                                                                - xs[index_control_motion[i]]) 
                         + p[i*par_joint + int(index[0]-1)*2+1]*(xs_normal[num_control + i] 
                                                                - xs[index_control_motion[i]+9]))
                
            else:
                tor[i] = (p[i*par_joint + int(index[1]-1)*2]*(xs_normal[i] 
                                                                - xs[index_control_motion[i]]) 
                         + p[i*par_joint + int(index[1]-1)*2+1]*(xs_normal[num_control + i] 
                                                                - xs[index_control_motion[i]+9]))
        return tor