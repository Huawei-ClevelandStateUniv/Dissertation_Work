#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  9 05:07:01 2019

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt

gender = 'M'
S = 4
T = 2

data_path = 'selected_data/' + gender + str(S) + '_S' + str(T) + '/'

motion = np.loadtxt(data_path + 'Motion_30000_40000_50HZ.txt')
belt = np.loadtxt(data_path + 'Belt_30000_40000_50HZ.txt')
InitL = np.loadtxt(data_path + 'InitialDataL_30000_40000_50HZ.txt')
InitR = np.loadtxt(data_path + 'InitialDataR_30000_40000_50HZ.txt')
Lstancet = np.loadtxt(data_path + 'Lstancet_30000_40000_50HZ.txt')
Lswingt = np.loadtxt(data_path + 'Lswingt_30000_40000_50HZ.txt')
Rstancet = np.loadtxt(data_path + 'Rstancet_30000_40000_50HZ.txt')
Rswingt = np.loadtxt(data_path + 'Rswingt_30000_40000_50HZ.txt')

Lstancex = np.loadtxt(data_path + 'Lstancex_30000_40000_50HZ.txt')
Lstancey = np.loadtxt(data_path + 'Lstancey_30000_40000_50HZ.txt')
Lswingx = np.loadtxt(data_path + 'Lswingx_30000_40000_50HZ.txt')
Lswingy = np.loadtxt(data_path + 'Lswingy_30000_40000_50HZ.txt')

Rstancex = np.loadtxt(data_path + 'Rstancex_30000_40000_50HZ.txt')
Rstancey = np.loadtxt(data_path + 'Rstancey_30000_40000_50HZ.txt')
Rswingx = np.loadtxt(data_path + 'Rswingx_30000_40000_50HZ.txt')
Rswingy = np.loadtxt(data_path + 'Rswingy_30000_40000_50HZ.txt')


fig = plt.figure(figsize=(12, 12))

ax1 = fig.add_subplot(2, 2, 1)
ax2 = fig.add_subplot(2, 2, 2)
ax3 = fig.add_subplot(2, 2, 3)
ax4 = fig.add_subplot(2, 2, 4)

for k in range(len(Lstancet[:, 0])-2):
    ax1.plot(Lstancet[k, :], Lstancex[k, :], 'r')
    ax1.plot(Rstancet[k, :], Rstancex[k, :], 'b')
    
    ax2.plot(Lstancet[k, :], Lstancey[k, :], 'r')
    ax2.plot(Rstancet[k, :], Rstancey[k, :], 'b')
    
    ax3.plot(Lswingt[k, :], Lswingx[k, :], 'r')
    ax3.plot(Rswingt[k, :], Rswingx[k, :], 'b')
    
    ax4.plot(Lswingt[k, :], Lswingy[k, :], 'r')
    ax4.plot(Rswingt[k, :], Rswingy[k, :], 'b')