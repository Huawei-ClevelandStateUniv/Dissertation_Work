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
import os

from record_load import load_record


show_nodes = 3000
#start_frame = 2000
start_frame = 56000

for j in range(9):
    for s in range(3):
        
        if j < 4:
            store_path = 'normal_walking/F'+str(j)+'_S'+str(s)
            if not os.path.exists(store_path):
                os.makedirs(store_path)
            
            moment_ID_name = 'raw_data/F'+str(j)+'_S'+str(s)+'/mocap_jointmoment.txt'
            save_mom = store_path +'/Moment_ID_'+str(start_frame)+'_'+str(show_nodes+start_frame)+'_100HZ.txt'
        
        else:
            
            store_path = 'normal_walking/M'+str(j-4)+'_S'+str(s)
            if not os.path.exists(store_path):
                os.makedirs(store_path)
            
            moment_ID_name = 'raw_data/M'+str(j-4)+'_S'+str(s)+'/mocap_jointmoment.txt'
            save_mom = store_path +'/Moment_ID_'+str(start_frame)+'_'+str(show_nodes+start_frame)+'_100HZ.txt'
            
        header, moment = load_record(moment_ID_name)
        
        moment_2d_names = ['LHipFlexion', 'LKneeFlexion', 'LAnklePlantarFlexion',
                           'RHipFlexion', 'RKneeFlexion', 'RAnklePlantarFlexion']
        
        index_2d_moment = []
        for i in range(len(moment_2d_names)):
            index_2d_moment.append(header.index(moment_2d_names[i]))
        
       
        chosen_moment = moment[start_frame:start_frame+show_nodes, index_2d_moment]
        
        chosen_moment[:, 1] = - chosen_moment[:, 1]
        chosen_moment[:, 2] = -chosen_moment[:, 2]
        
        chosen_moment[:, 4] = - chosen_moment[:, 4]
        chosen_moment[:, 5] = -chosen_moment[:, 5]
        
        with open(save_mom, 'w') as outfile:
            StringP = ""
            for ii in range(show_nodes):
                for jj in range(len(moment_2d_names)):
                    StringP += str(chosen_moment[ii, jj])
                    StringP += " "
                StringP += "\n"
            outfile.write(StringP)