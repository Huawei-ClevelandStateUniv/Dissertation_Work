#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 28 11:46:45 2019



@author: huawei
"""

import numpy as np

def event_detection(file_name, event_sign):
    
    # load data line by line
    with open(file_name, 'r') as the_file:
        all_data = [line.strip() for line in the_file.readlines()]
    
    time_frames = np.zeros(len(event_sign))
    
    time0 = float(all_data[1].split('\t')[0])
    
    # assign separate data into predefined motion capture data
    for i in range(1, len(all_data)-10):
        line = all_data[i].split('\t')
           
        for j in range(len(event_sign)):
            if line[0][:9] == '# EVENT ' + event_sign[j]:
                time_frames[j] = all_data[i+2].split('\t')[0]
    return time0, time_frames
    
    
    
    
    
    
    