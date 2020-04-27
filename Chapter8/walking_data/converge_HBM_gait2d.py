#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 28 12:01:35 2019

@author: huawei
"""

import numpy as np

def HBM_to_gait2d_motion_converge(motion, frames, pelvis_motion, grf):
    
    motion_2d = np.zeros((frames, 18))
    grf_2d = np.zeros((frames, 6))
        #show_motion_2d_50 = np.zeros((int(show_nodes/2), 18))
        
    motion_2d[:, 0] = motion[:, 1]
    motion_2d[:, 1] = motion[:, 2]
    motion_2d[:, 2] = -motion[:, 3] - pelvis_motion
    motion_2d[:, 3] = motion[:, 4] + motion[:, 3]
    motion_2d[:, 4] = -motion[:, 5]
    motion_2d[:, 5] = -motion[:, 6] - np.pi/2 + 12/180*np.pi
    motion_2d[:, 6] = motion[:, 7] + motion[:, 3]
    motion_2d[:, 7] = -motion[:, 8] 
    motion_2d[:, 8] = -motion[:, 9] - np.pi/2 + 12/180*np.pi

    motion_2d[1:, 9:18] = (motion_2d[1:, :9] - motion_2d[:-1, :9])/0.01
    
    grf_2d[:, 0] = -grf[:, 0]
    grf_2d[:, 1] = grf[:, 1]
    grf_2d[:, 2] = -grf[:, 2]
    grf_2d[:, 3] = -grf[:, 3]
    grf_2d[:, 4] = grf[:, 4]
    grf_2d[:, 5] = -grf[:, 5]
    
    return motion_2d, grf_2d