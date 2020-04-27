#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  7 15:01:50 2019

@author: huawei
"""

'''
This code generate swing/stance path data from motion data, not marker data
'''

import numpy as np
from record_load import load_record
from gait_separation import sep_gait_by_force
import os
import yaml
from simulate_auto import map_values_to_autolev_symbols
from motion2d_extraction import extract_gait2d


gender = 'F'

for S in range(2, 3):
    for T in range(1, 2):
        
        st_walking = 30000
        ed_walking = 40000
        ext_nodes = 100
        delta = 0.02
        every_node = int(delta/0.01)
        num_nodes = int((ed_walking - st_walking)/every_node) + 2*ext_nodes
        ext_gait = 1
        
        row_path = 'raw_data/'+gender + str(S) + '_S' + str(T) + '/'
        processed_path = 'processed_data/'+gender + str(S) + '_S' + str(T) + '/'
        store_path = 'selected_data/'+gender + str(S) + '_S' + str(T) + '/'
        if not os.path.exists(store_path):
            os.makedirs(store_path)
            
        with open('processed_data/'+gender+str(S)+'_S'+str(T)+'/Model_Parameters_2.yml', 'r') as f:
            constants_dict = yaml.load(f)
        constants_dict['kc'] = 1e5
        constants_dict['ce'] = 0.005
        constants_dict['fyd'] = -0.09
        constants_dict['fyg'] = -0.09
        const_dict = map_values_to_autolev_symbols(constants_dict)
            
        motion_3d_data = np.loadtxt(processed_path + 'inverse_kinematics_dof.txt')
        grf_data = np.loadtxt(processed_path + 'grf.txt')
        belt_data = load_record(row_path + 'record.txt')
        
        index_delta_motion = np.linspace(st_walking-2*ext_nodes, ed_walking+2*ext_nodes,
                                       int((ed_walking-st_walking)/every_node)+2*ext_nodes,
                                       endpoint=False, dtype=np.int32)
        
        index_delta_belt = np.linspace(st_walking, ed_walking, int((ed_walking-st_walking)/every_node),
                                       endpoint=False, dtype=np.int32)
        
        joints_2d_index = np.array([1, 3, 7, 34, 40, 42, 33, 39, 41])
        
        motion_2d = np.zeros((int((ed_walking-st_walking)/every_node+2*ext_nodes), 18))
        motion_2d[:, :9] = extract_gait2d(motion_3d_data, index_delta_motion)
        motion_2d[1:, 9:] = (motion_2d[1:, :9] - motion_2d[:-1, :9])/delta
        
        grf_2d = grf_data[index_delta_motion, :]
        
        belt_2d = np.zeros((int((ed_walking-st_walking)/every_node), 2))
        belt_2d[:, 0] = np.interp(motion_3d_data[index_delta_belt, 0], belt_data[:, 0], belt_data[:, 1])
        belt_2d[:, 1] = np.interp(motion_3d_data[index_delta_belt, 0], belt_data[:, 0], belt_data[:, 2])
        
        
        sep_gait_by_force(motion_2d, grf_2d, st_walking, ed_walking, num_nodes, ext_nodes,
                          ext_gait, delta, const_dict, plot_sign=False, write_sign=True,
                          store_path=store_path)
        
        
        
        with open(store_path+'Motion_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(len(motion_2d[:, 0])-2*ext_nodes):
                for g in range(len(motion_2d[0, :])):
                    StringP += str(motion_2d[r+ext_nodes, g])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
        
        with open(store_path+'Belt_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(len(belt_2d[:, 0])):
                for g in range(len(belt_2d[0, :])):
                    StringP += str(belt_2d[r, g])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)