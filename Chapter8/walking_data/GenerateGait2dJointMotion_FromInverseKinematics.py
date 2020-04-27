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
import yaml
from annimation_Open import animate_pendulum
from gait2dpi_dGRF_LinContact import evaluate_autolev_rhs as autolev_rhs
from simulate import map_values_to_autolev_symbols

show_nodes = 10000

for j in range(9):
    for s in range(3):
        
        if j < 4:
            motion_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/inverse_kinematics_dof.txt'
            save_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/Motion_IK_30000_40000_100HZ.txt'
            save_name_50 = 'processed_data/F'+str(j)+'_S'+str(s)+'/Motion_IK_30000_40000_50HZ.txt'
            belt_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/Belt_30000_40000_100HZ.txt'
            para_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/Model_Parameters.yml'
            video_save_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/Sample_motion_IK_NewParam.mp4'
        else:
            motion_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/inverse_kinematics_dof.txt'
            save_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/Motion_IK_30000_40000_100HZ.txt'
            save_name_50 = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/Motion_IK_30000_40000_50HZ.txt'
            belt_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/Belt_30000_40000_100HZ.txt'
            para_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/Model_Parameters.yml'
            video_save_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/Sample_motion_IK_NewParam.mp4'
            
        motion = np.loadtxt(motion_name)
        show_belt = np.loadtxt(belt_name)
        index_2d_dof = np.array([0, 1, 3, 7, 34, 40, 42, 33, 39, 41], dtype=np.int32)
        
        start_frame = 30000
        
        show_motion = motion[start_frame:start_frame+show_nodes, index_2d_dof]
        show_pelvis = motion[start_frame:start_frame+show_nodes, 5]
        
        show_motion_2d = np.zeros((show_nodes, 18))
        show_motion_2d_50 = np.zeros((int(show_nodes/2), 18))
        
        show_motion_2d[:, 0] = show_motion[:, 1]
        show_motion_2d[:, 1] = show_motion[:, 2]
        show_motion_2d[:, 2] = -show_motion[:, 3] - show_pelvis
        show_motion_2d[:, 3] = show_motion[:, 4] + show_motion[:, 3]
        show_motion_2d[:, 4] = -show_motion[:, 5]
        show_motion_2d[:, 5] = -show_motion[:, 6] - np.pi/2 + 12/180*np.pi
        show_motion_2d[:, 6] = show_motion[:, 7] + show_motion[:, 3]
        show_motion_2d[:, 7] = -show_motion[:, 8] 
        show_motion_2d[:, 8] = -show_motion[:, 9] - np.pi/2 + 12/180*np.pi

        show_motion_2d[1:, 9:18] = (show_motion_2d[1:, :9] - show_motion_2d[:-1, :9])/0.01
        
        for k in range(int(show_nodes/2)):
            show_motion_2d_50[k, :9] = show_motion_2d[2*k, :9]
            if k > 0:
                show_motion_2d_50[k, 9:] = (show_motion_2d[2*k, :9] - show_motion_2d[2*(k-1), :9])/0.02
        
        with open(save_name, 'w') as outfile:
            StringP = ""
            for ii in range(show_nodes):
                for jj in range(18):
                    StringP += str(show_motion_2d[ii, jj])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
        with open(save_name_50, 'w') as outfile:
            StringP = ""
            for kk in range(int(show_nodes/2)):
                for pp in range(18):
                    StringP += str(show_motion_2d_50[kk, pp])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
            
        show_nodes_plot = 1001
        
        with open(para_name, 'r') as f:
            subj_info = yaml.load(f)
            
        constants_dict = map_values_to_autolev_symbols(subj_info)
            
        sticks = np.zeros((show_nodes_plot, 20))
        belt_disp = np.zeros(show_nodes_plot)
        
        for qq in range(show_nodes_plot):
            
            x = show_motion_2d[qq, :9]
            xd = np.zeros(9)
            xdd = np.zeros(9)
            vs = show_belt[qq, :]
            
            if qq > 0:
                belt_disp[qq] = belt_disp[qq-1] - (show_belt[qq, 0] + show_belt[qq-1, 0])/2*0.01 - 0.8*0.01
            
            _, _, _, _, _, _, _, sticks[qq, :] = \
                                autolev_rhs(x, xd, xdd, vs, constants_dict)
                                
        t = show_motion[:show_nodes_plot, 0] - show_motion[0, 0]
        animate_pendulum(t, sticks, belt_disp, filename = video_save_name)
        
        
        
        
        
        
