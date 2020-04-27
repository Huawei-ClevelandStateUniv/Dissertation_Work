#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 24 08:59:32 2020

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt

Subj = 3
Trail = 1

start_nodes = 8000
end_nodes = 13001

folderE = ('Data/2DoF Data Simplify/Subj0'+str(Subj)+'/')

PD_path = 'PD/Analysis_Paper/Subj0'+str(Subj)+'/Pert'+str(Trail)+'_GFL_SPDR_Eps2_Noise025/'
FPD_path = 'FPD/Analysis_Paper/Subj0'+str(Subj)+'/Pert'+str(Trail)+'_GFL_FPD_Eps3_Noise025/'
FPDTD_path = 'GoodworthCon/Analysis_Paper/Subj0'+str(Subj)+'Pert'+str(Trail)+'GoodWorth5/'
LSCTD_path = 'LinCombDelay/Analysis_Paper/Subj0'+str(Subj)+'Pert'+str(Trail)+'_GFL_LinComb_delay60/'
NN_path = 'NN14/Analysis_Paper/Subj0'+str(Subj)+'Pert'+str(Trail)+'_GFL_NN14/'
NNTD_path = 'NN18Delay/Analysis_Paper/Subj0'+str(Subj)+'Pert'+str(Trail)+'NN18Delay/'


motion_PD = np.loadtxt(PD_path + 'Best_Traj.txt')*180/np.pi
motion_FPD = np.loadtxt(FPD_path + 'Best_Traj.txt')*180/np.pi
motion_FPDTD = np.loadtxt(FPDTD_path + 'Best_Traj.txt')*180/np.pi
motion_LSCTD = np.loadtxt(LSCTD_path + 'Best_Traj.txt')*180/np.pi
motion_NN = np.loadtxt(NN_path + 'Best_Traj.txt')*180/np.pi
motion_NNTD = np.loadtxt(NNTD_path + 'Best_Traj.txt')*180/np.pi

x_meas = np.loadtxt(folderE + 'JointsGFL000' + str(Trail+1) + '.txt',
                    usecols=(1, 2, 3, 4))[start_nodes:end_nodes, :]
pert_meas = np.loadtxt(folderE + 'SwayL000' + str(Trail+1) + '.txt',
                      usecols=(1))[start_nodes:end_nodes]

time = np.loadtxt(folderE + 'JointsGFL000' + str(Trail+1) + '.txt',
                    usecols=(0))[start_nodes:end_nodes]

time = time - time[0]

fig1 = plt.figure(figsize = (8, 14))
ax1 = fig1.add_subplot(7, 2, 1)
plt.title('Ankle Joint Motion')
plt.ylabel('Perturbation (cm)')
ax1.set_xticklabels([])
ax2 = fig1.add_subplot(7, 2, 2)
plt.title('Hip Joint Motion')
ax2.set_xticklabels([])
ax3 = fig1.add_subplot(7, 2, 3)
plt.ylabel('PD (deg)')
ax3.set_xticklabels([])
ax4 = fig1.add_subplot(7, 2, 4)
ax4.set_xticklabels([])
ax5 = fig1.add_subplot(7, 2, 5)
plt.ylabel('FPD (deg)')
ax5.set_xticklabels([])
ax6 = fig1.add_subplot(7, 2, 6)
ax6.set_xticklabels([])
ax7 = fig1.add_subplot(7, 2, 7)
plt.ylabel('FPDTD (deg)')
ax7.set_xticklabels([])
ax8 = fig1.add_subplot(7, 2, 8)
ax8.set_xticklabels([])
ax9 = fig1.add_subplot(7, 2, 9)
plt.ylabel('LSCTD (deg)')
ax9.set_xticklabels([])
ax10 = fig1.add_subplot(7, 2, 10)
ax10.set_xticklabels([])
ax11 = fig1.add_subplot(7, 2, 11)
plt.ylabel('NN (deg)')
ax11.set_xticklabels([])
ax12 = fig1.add_subplot(7, 2, 12)
ax12.set_xticklabels([])
ax13 = fig1.add_subplot(7, 2, 13)
plt.ylabel('NNTD (deg)')
plt.xlabel('Time (s)')
ax14 = fig1.add_subplot(7, 2, 14)
plt.xlabel('Time (s)')

ax1.plot(time, pert_meas*100)
ax2.plot(time, pert_meas*100)
ax3.plot(time, x_meas[:, 0], label = 'Measured motion')
ax3.plot(time, motion_PD[:5001, 0], label = 'Identified motion')
# ax3.plot(time, motion_PD[5001:5001*2, 0], label = 'Identificed motion in episode 2')

ax4.plot(time, x_meas[:, 1], label = 'Measured motion')
ax4.plot(time, motion_PD[:5001, 1], label = 'Identified motion')
#ax4.plot(time, motion_PD[5001:5001*2, 1], label = 'Identificed motion in episode 2')

ax5.plot(time, x_meas[:, 0], label = 'Measured motion')
ax5.plot(time, motion_FPD[:5001, 0], label = 'Identified motion')
#ax5.plot(time, motion_FPD[5001:5001*2, 0], label = 'Identificed motion in episode 2')
#ax5.plot(time, motion_FPD[5001*2:5001*3, 0], label = 'Identificed motion in episode 3')

ax6.plot(time, x_meas[:, 1], label = 'Measured motion')
ax6.plot(time, motion_FPD[:5001, 1], label = 'Identified motion')
#ax6.plot(time, motion_FPD[5001:5001*2, 1], label = 'Identificed motion in episode 2')
#ax6.plot(time, motion_FPD[5001*2:5001*3, 1], label = 'Identificed motion in episode 3')

ax7.plot(time, x_meas[:, 0], label = 'Measured motion')
ax7.plot(time, motion_FPDTD[:5001, 0], label = 'Identified motion')

ax8.plot(time, x_meas[:, 1], label = 'Measured motion')
ax8.plot(time, motion_FPDTD[:5001, 1], label = 'Identified motion')

ax9.plot(time, x_meas[:, 0], label = 'Measured motion')
ax9.plot(time, motion_LSCTD[:5001, 0], label = 'Identified motion')

ax10.plot(time, x_meas[:, 1], label = 'Measured motion')
ax10.plot(time, motion_LSCTD[:5001, 1], label = 'Identified motion')

ax11.plot(time, x_meas[:, 0], label = 'Measured motion')
ax11.plot(time, motion_NN[:5001, 0], label = 'Identified motion')

ax12.plot(time, x_meas[:, 1], label = 'Measured motion')
ax12.plot(time, motion_NN[:5001, 1], label = 'Identified motion')

ax13.plot(time, x_meas[:, 0], label = 'Measured motion')
ax13.plot(time, motion_NNTD[:5001, 0], label = 'Identified motion')

ax14.plot(time, x_meas[:, 1], label = 'Measured motion')
ax14.plot(time, motion_NNTD[:5001, 1], label = 'Identified motion')

plt.legend(loc='lower left', bbox_to_anchor=(1, 1))




