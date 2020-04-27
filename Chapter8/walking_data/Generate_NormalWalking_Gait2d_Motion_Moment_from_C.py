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
from record_load import load_record

skip_frames = 1500
save_frames = 3000

for j in range(6, 7):
    for s in range(1, 2):
        
        if j < 4:
            store_path = 'normal_walking/F'+str(j)+'_S'+str(s)
            if not os.path.exists(store_path):
                os.makedirs(store_path)
            
            record_name = 'raw_data/F'+str(j)+'_S'+str(s)+'/record.txt'
            motion_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/mocap_dof.txt'
            moment_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/mocap_jointmoment.txt'
            grf_name = 'processed_data/F'+str(j)+'_S'+str(s)+'/grf.txt'

        else:
            
            store_path = 'normal_walking/M'+str(j-4)+'_S'+str(s)
            if not os.path.exists(store_path):
                os.makedirs(store_path)
                
            record_name = 'raw_data/M'+str(j-4)+'_S'+str(s)+'/record.txt'
            motion_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/mocap_dof.txt'
            moment_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/mocap_jointmoment.txt'
            grf_name = 'processed_data/M'+str(j-4)+'_S'+str(s)+'/grf.txt'
            
        save_dof1 = store_path +'/Motion_normalwalking_1_100HZ.txt'
        save_mom1 = store_path +'/Moment_normalwalking_1_100HZ.txt'
        save_grf1 = store_path +'/GRF_nromalwalking_1_100HZ.txt'
        save_dof2 = store_path +'/Motion_normalwalking_2_100HZ.txt'
        save_mom2 = store_path +'/Moment_normalwalking_2_100HZ.txt'
        save_grf2 = store_path +'/GRF_nromalwalking_2_100HZ.txt'
        
        save_pelvis1 = store_path +'/Pelvis_nromalwalking_1_100HZ.txt'
        save_pelvis2 = store_path +'/Pelvis_nromalwalking_2_100HZ.txt'

        #motion = np.loadtxt(motion_name)
        grf = np.loadtxt(grf_name)
        
        event_sign = ['C', 'E']
        time_index = event_detection(record_name, event_sign)
        event_index = []
        
        motion_2d_names = ['TimeStamp', 'PelvisX', 'PelvisZ', 'TrunkFlexion', 'LHipFlexion', 'LKneeFlexion', 'LAnklePlantarFlexion',
                           'RHipFlexion', 'RKneeFlexion', 'RAnklePlantarFlexion']
        
        header, motion = load_record(motion_name)
        index_2d_dof = []
        for i in range(len(motion_2d_names)):
            index_2d_dof.append(header.index(motion_2d_names[i]))
            
        index_2d_pelvis = header.index('PelvisForwardPitch')
        
        #index_2d_dof = np.array([0, 1, 3, 7, 34, 40, 42, 33, 39, 41], dtype=np.int32)
        index_2d_grf = np.array([2, 1, 3, 8, 7, 9])
        
        for m in range(len(event_sign)):
            event_index.append((np.abs(motion[:, 0] - time_index[m])).argmin())
            
        HBM_motion1 = motion[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, index_2d_dof]
        HBM_pelvis1 = motion[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, 5]
        HBM_grf1 = grf[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, index_2d_grf]
        
        HBM_motion2 = motion[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, index_2d_dof]
        HBM_pelvis2 = motion[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, 5]
        HBM_grf2 = grf[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, index_2d_grf]
        
        motion_2d1, grf_2d1 = HBM_to_gait2d_motion_converge(HBM_motion1, save_frames, HBM_pelvis1, HBM_grf1)
        motion_2d2, grf_2d2 = HBM_to_gait2d_motion_converge(HBM_motion2, save_frames, HBM_pelvis2, HBM_grf2)
        
        
        header, moment = load_record(moment_name)
        
        moment_2d_names = ['LHipFlexion', 'LKneeFlexion', 'LAnklePlantarFlexion',
                           'RHipFlexion', 'RKneeFlexion', 'RAnklePlantarFlexion']
        
        pelvis_names = ['PelvisX',	'PelvisY',	'PelvisZ',	'PelvisYaw',	'PelvisForwardPitch',	'PelvisRightRoll',	'TrunkFlexion']
        
        index_2d_moment = []
        for i in range(len(moment_2d_names)):
            index_2d_moment.append(header.index(moment_2d_names[i]))
            
        index_pelvis = []
        for i in range(len(pelvis_names)):
            index_pelvis.append(header.index(pelvis_names[i]))
        
       
        moment_2d1 = moment[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, index_2d_moment]
        moment_2d2 = moment[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, index_2d_moment]
        
        moment_pelvis1 = moment[event_index[0] + skip_frames:event_index[0] + skip_frames + save_frames, index_pelvis]
        moment_pelvis2 = moment[event_index[1] + skip_frames:event_index[1] + skip_frames + save_frames, index_pelvis]
        
        moment_2d1[:, 1] = - moment_2d1[:, 1]
        moment_2d1[:, 2] = -moment_2d1[:, 2]
        moment_2d1[:, 4] = - moment_2d1[:, 4]
        moment_2d1[:, 5] = -moment_2d1[:, 5]
        
        moment_2d2[:, 1] = - moment_2d2[:, 1]
        moment_2d2[:, 2] = -moment_2d2[:, 2]
        moment_2d2[:, 4] = - moment_2d2[:, 4]
        moment_2d2[:, 5] = -moment_2d2[:, 5]
        
        
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
            
        with open(save_mom1, 'w') as outfile:
            StringP = ""
            for ii in range(save_frames):
                for jj in range(6):
                    StringP += str(moment_2d1[ii, jj])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
        with open(save_mom2, 'w') as outfile:
            StringP = ""
            for ii in range(save_frames):
                for jj in range(6):
                    StringP += str(moment_2d2[ii, jj])
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
            
            
        with open(save_pelvis1, 'w') as outfile:
            StringP = ""
            for kk in range(save_frames):
                for pp in range(7):
                    StringP += str(moment_pelvis1[kk, pp])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)
            
        with open(save_pelvis2, 'w') as outfile:
            StringP = ""
            for kk in range(save_frames):
                for pp in range(7):
                    StringP += str(moment_pelvis2[kk, pp])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)