#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 20 11:14:50 2018

# this code is to plot R^2 fit of different controller structure.

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
#from matplotlib import xticklabels

Data_path = 'Data/2DoF Data Simplify/'
PD_path = 'PD/Analysis_Paper/'
FPD_path = 'FPD/Analysis_Paper/'
Goodworth_path = 'GoodworthCon/Analysis_Paper/'
LinComb_path = 'LinCombDelay/Analysis_Paper/'
NN14_path = 'NN14/Analysis_Paper/'
NN_path = 'NN18Delay/Analysis_Paper/'

Data_rms_mean = np.loadtxt(Data_path + 'Data_RMS.txt')
PD_rms_mean = np.loadtxt(PD_path + 'Lowest_RMS.txt')
FPD_rms_mean = np.loadtxt(FPD_path + 'Lowest_RMS.txt')
GoodWorth_rms_mean = np.loadtxt(Goodworth_path + 'Lowest_RMS.txt')
LinComb_rms_mean = np.loadtxt(LinComb_path + 'Lowest_RMS.txt')
NN14_mean = np.loadtxt(NN14_path + 'Lowest_RMS.txt')
NN_rms_mean = np.loadtxt(NN_path + 'Lowest_RMS.txt')

NN_rms_mean_1_fliter = NN_rms_mean[~np.isnan(NN_rms_mean[:, 0]), 0]
NN_rms_mean_2_fliter = NN_rms_mean[~np.isnan(NN_rms_mean[:, 1]), 1]

rms_mean_1 = [Data_rms_mean[:, 0]*180/np.pi, PD_rms_mean[:, 0]*180/np.pi, FPD_rms_mean[:, 0]*180/np.pi,
              GoodWorth_rms_mean[:, 0]*180/np.pi, LinComb_rms_mean[:, 0]*180/np.pi,
              NN14_mean[:, 0]*180/np.pi, NN_rms_mean_1_fliter*180/np.pi]

rms_mean_2 = [Data_rms_mean[:, 1]*180/np.pi, PD_rms_mean[:, 1]*180/np.pi, FPD_rms_mean[:, 1]*180/np.pi,
              GoodWorth_rms_mean[:, 1]*180/np.pi, LinComb_rms_mean[:, 1]*180/np.pi,
              NN14_mean[:, 1]*180/np.pi, NN_rms_mean_2_fliter*180/np.pi]


ind1 = np.arange(7)

fig1 = plt.figure(figsize=(9,4))
ax1 = fig1.add_subplot(1,1,1)
#plt.title('Root Mean Sqaure (RMS) of the fit')
plt.plot([-0 -0], [-0 -0],'b-', label='Perturbation Trial 1')
plt.plot([-0 -0], [-0 -0],'g-', label='Perturbation Trial 2')
plt.ylabel('RMS fit error (deg)')
plt.legend()

ax1.plot([ind1[0]-1, ind1[-1]+1], [0, 0], '--', color = 'darkgrey', zorder=0)

bp1_1 = plt.boxplot(rms_mean_1, positions = ind1 - 0.2, widths = 0.2)
bp2_1 = plt.boxplot(rms_mean_2, positions = ind1 + 0.2, widths = 0.2)


plt.xlim([-1, len(ind1)])
ax1.set_xticklabels(['Data', 'PD', 'FPD', 'FPDTD', 'LSCTD', 'NN', 'NNTD'])
ax1.set_xticks(list(ind1))

plt.setp(bp1_1['boxes'], color='blue')
plt.setp(bp1_1['caps'], color='blue')


plt.setp(bp2_1['boxes'], color='green')
plt.setp(bp2_1['caps'], color='green')