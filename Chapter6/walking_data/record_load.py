#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  7 15:37:59 2019

@author: huawei
"""
import sys
sys.path.append('../')

import numpy as np

def load_record(file_name):

    # load data line by line
    with open(file_name, 'r') as the_file:
        all_data = [line.strip() for line in the_file.readlines()]
    
    head_line = all_data[0].split('\t')  # separate headline
    col = len(head_line)
    
    data_matrix = np.zeros((len(all_data), col))
    
    index = 0
    # assign separate data into predefined motion capture data
    for i in range(1, len(all_data)):
        line = all_data[i].split('\t')
        if line[0][0] == '#':
            continue
        else:   
            for j in range(col):
                data_matrix[index, j] = float(line[j])
            index += 1
            
    index = np.where(data_matrix[:, 0]!=0)
    
    nonzero_data_matrix = data_matrix[index[0], :]
            
    return head_line, nonzero_data_matrix
            