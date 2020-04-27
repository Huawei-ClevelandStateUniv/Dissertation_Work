#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  8 22:32:02 2019

@author: huawei
"""

import numpy as np
from numpy import sin, cos

def get_hip_point(theta_t, theta_h, theta_k, Lthigh, Lshank):
    
    hip_point = np.zeros(2)
    
    hip_point[0] =  -Lthigh*sin(theta_t + theta_h) - Lshank*sin(theta_t + theta_h + theta_k)
    hip_point[1] =  Lthigh*cos(theta_t + theta_h) + Lshank*cos(theta_t + theta_h + theta_k)
    
    return hip_point


def get_ankle_point(theta_t, theta_h, theta_k, Lthigh, Lshank):
    
    ankle_point = np.zeros(2)
        
    ankle_point[0] =  Lthigh*sin(theta_t + theta_h) + Lshank*sin(theta_t + theta_h + theta_k)
    ankle_point[1] =  -Lthigh*cos(theta_t + theta_h) - Lshank*cos(theta_t + theta_h + theta_k)
    
    return ankle_point