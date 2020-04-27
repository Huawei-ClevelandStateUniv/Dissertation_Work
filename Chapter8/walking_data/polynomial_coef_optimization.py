#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 18:27:38 2019

@author: huawei
"""

import numpy as np
from ScaledPolynomial_Stancey import get_stancey_polynomial
from ScaledPolynomial_Stancex import get_stancex_polynomial
from ScaledPolynomial_Swingx import get_swingx_polynomial
from ScaledPolynomial_Swingy import get_swingy_polynomial
import os

S = 0
T = 0

num_par = 5
repeat = 5

for S in range(9):
    for T in range(0, 1):
        
        if S < 4:

            data_path = ('selected_data/F'+str(S)+'_S'+str(T))
            store_path = 'swing_stance_path/F'+ str(S)+'_S'+str(T)
            
        else:
            data_path = ('selected_data/M'+str(S-4)+'_S'+str(T))
            store_path = 'swing_stance_path/M'+ str(S-4)+'_S'+str(T)
        
        if not os.path.exists(store_path):
            os.makedirs(store_path)
        
        Lstancex = np.loadtxt(data_path + '/Lstancex_30000_40000_50HZ.txt')
        Rstancex = np.loadtxt(data_path + '/Rstancex_30000_40000_50HZ.txt')
        Lstancey = np.loadtxt(data_path + '/Lstancey_30000_40000_50HZ.txt')
        Rstancey = np.loadtxt(data_path + '/Rstancey_30000_40000_50HZ.txt')
        Lstancet = np.loadtxt(data_path + '/Lstancet_30000_40000_50HZ.txt')
        Rstancet = np.loadtxt(data_path + '/Rstancet_30000_40000_50HZ.txt')
        
        get_stancex_polynomial(Lstancex, Rstancex, Lstancet, Rstancet, num_par,
                                   repeat, store_path)
        get_stancey_polynomial(Lstancey, Rstancey, Lstancet, Rstancet, num_par,
                                   repeat, store_path)
        
        
        Lswingx = np.loadtxt(data_path + '/Lswingx_30000_40000_50HZ.txt')
        Rswingx = np.loadtxt(data_path + '/Rswingx_30000_40000_50HZ.txt')
        Lswingy = np.loadtxt(data_path + '/Lswingy_30000_40000_50HZ.txt')
        Rswingy = np.loadtxt(data_path + '/Rswingy_30000_40000_50HZ.txt')
        Lswingt = np.loadtxt(data_path + '/Lswingt_30000_40000_50HZ.txt')
        Rswingt = np.loadtxt(data_path + '/Rswingt_30000_40000_50HZ.txt')
        
        get_swingx_polynomial(Lswingx, Rswingx, Lswingt, Rswingt, num_par,
                                   repeat, store_path)
        get_swingy_polynomial(Lswingy, Rswingy, Lswingt, Rswingt, num_par,
                                   repeat, store_path)