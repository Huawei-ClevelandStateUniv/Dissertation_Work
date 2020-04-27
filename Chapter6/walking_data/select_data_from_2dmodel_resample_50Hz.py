#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  1 13:15:06 2019

This code is to select walking data from 30000 to 40000 data frames and then 
resampled as 50Hz.

@author: huawei
"""

import numpy as np

for sub in range(9):
    for tri in range(3):
        
        if sub < 4:
            gender = 'F'
            subj = sub
        else:
            gender = 'M'
            subj = sub-4

        data_path = 'joint_motion_2d_model/'+gender+str(subj)+'_S'+str(tri)+'/'
        
        motion_all = np.loadtxt(data_path + 'joint_motion.txt')
        moment_all = np.loadtxt(data_path + 'joint_moment.txt')
        grf_all = np.loadtxt(data_path + 'grf.txt')
        
        select_start = 30000
        select_end = 40000
        
        select_index = np.linspace(select_start, select_end, int((select_end-select_start)/2 + 1), dtype = int)
        
        motion_select = motion_all[select_index, :]
        moment_select = moment_all[select_index, :]
        grf_select = grf_all[select_index, :]
        
        store_path = 'selected_data/'+gender+str(subj)+'_S'+str(tri)+'/'
        with open(store_path + 'Motion_2dmodel_30000_40000_50HZ.txt', 'w') as outfile:
            StringP = ''
            for i in range(len(motion_select[:, 0])):
                for j in range(len(motion_select[0, :])):
                    StringP += str(motion_select[i, j])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path + 'Moment_2dmodel_30000_40000_50HZ.txt', 'w') as outfile:
            StringP = ''
            for i in range(len(moment_select[:, 0])):
                for j in range(len(moment_select[0, :])):
                    StringP += str(moment_select[i, j])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path + 'Grf_2dmodel_30000_40000_50HZ.txt', 'w') as outfile:
            StringP = ''
            for i in range(len(grf_select[:, 0])):
                for j in range(len(grf_select[0, :])):
                    StringP += str(grf_select[i, j])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)