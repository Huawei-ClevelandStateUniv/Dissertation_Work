#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  9 04:48:10 2019

@author: huawei
"""

import numpy as np


def extract_gait2d(motion, row_index):
    
    show_motion_2d = np.zeros((len(row_index), 9))
    
    index_2d_dof = np.array([1, 3, 7, 34, 40, 42, 33, 39, 41], dtype=np.int32)
    
    show_motion_2d[:, 0] = motion[row_index, index_2d_dof[0]]
    show_motion_2d[:, 1] = motion[row_index, index_2d_dof[1]]
    show_motion_2d[:, 2] = -motion[row_index, index_2d_dof[2]] - motion[row_index, 5]
    show_motion_2d[:, 3] = motion[row_index, index_2d_dof[3]] + motion[row_index, index_2d_dof[2]]
    show_motion_2d[:, 4] = -motion[row_index, index_2d_dof[4]]
    show_motion_2d[:, 5] = -motion[row_index, index_2d_dof[5]] - np.pi/2 + 12/180*np.pi
    show_motion_2d[:, 6] = motion[row_index, index_2d_dof[6]] + motion[row_index, index_2d_dof[2]]
    show_motion_2d[:, 7] = -motion[row_index, index_2d_dof[7]] 
    show_motion_2d[:, 8] = -motion[row_index, index_2d_dof[8]] - np.pi/2 + 12/180*np.pi

    return show_motion_2d