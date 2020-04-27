#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 27 13:56:23 2018

@author: huawei
"""

'''

show generated joint motions

'''

import sys
sys.path.append('../')

import numpy as np
from event_detection import event_detection
from converge_HBM_gait2d import HBM_to_gait2d_motion_converge
import os

skip_frames = 1500
save_frames = 3000

for j in range(9):
    for s in range(3):
        
        if j < 4:
            store_path = 'normal_walking/F'+str(j)+'_S'+str(s)
            if not os.path.exists(store_path):
                os.makedirs(store_path)
            
            record_name = 'raw_data/F'+str(j)+'_S'+str(s)+'/record.txt'
            motion_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/inverse_kinematics_dof.txt'
            grf_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/grf.txt'

        else:
            
            store_path = 'normal_walking/M'+str(j-4)+'_S'+str(s)
            if not os.path.exists(store_path):
                os.makedirs(store_path)
                
            record_name = 'raw_data/M'+str(j-4)+'_S'+str(s)+'/record.txt'
            motion_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/inverse_kinematics_dof.txt'
            grf_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/grf.txt'
            
        save_dof1 = store_path +'/Motion_normalwalking_1_100HZ.txt'
        save_grf1 = store_path +'/GRF_nromalwalking_1_100HZ.txt'
        save_dof2 = store_path +'/Motion_normalwalking_2_100HZ.txt'
        save_grf2 = store_path +'/GRF_nromalwalking_2_100HZ.txt'

            
        motion = np.loadtxt(motion_name)
        grf = np.loadtxt(grf_name)
        
        event_sign = ['C', 'E']
        time_index = event_detection(record_name, event_sign)
        
        event_index = []
        
        for m in range(len(event_sign)):
            
            event_index.append((np.abs(motion[:, 0] - time_index[m])).argmin())
            
        
        index_2d_dof = np.array([0, 1, 3, 7, 34, 40, 42, 33, 39, 41], dtype=np.int32)
        index_2d_grf = np.array([2, 1, 3, 8, 7, 9])
        
        HBM_motion1 = motion[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, index_2d_dof]
        HBM_pelvis1 = motion[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, 5]
        HBM_grf1 = grf[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, index_2d_grf]
        
        HBM_motion2 = motion[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, index_2d_dof]
        HBM_pelvis2 = motion[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, 5]
        HBM_grf2 = grf[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, index_2d_grf]
        
        motion_2d1, grf_2d1 = HBM_to_gait2d_motion_converge(HBM_motion1, save_frames, HBM_pelvis1, HBM_grf1)
        motion_2d2, grf_2d2 = HBM_to_gait2d_motion_converge(HBM_motion2, save_frames, HBM_pelvis2, HBM_grf2)
        
        with open(save_dof1, 'w') as outfile:
            StringP = ""
            for ii in range(save_frames):
                for jj in range(18):
                    StringP += str(motion_2d1[ii, jj])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
        with open(save_dof2, 'w') as outfile:
            StringP = ""
            for ii in range(save_frames):
                for jj in range(18):
                    StringP += str(motion_2d2[ii, jj])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
        with open(save_grf1, 'w') as outfile:
            StringP = ""
            for kk in range(save_frames):
                for pp in range(6):
                    StringP += str(HBM_grf1[kk, pp])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
        with open(save_grf2, 'w') as outfile:
            StringP = ""
            for kk in range(save_frames):
                for pp in range(6):
                    StringP += str(HBM_grf2[kk, pp])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)