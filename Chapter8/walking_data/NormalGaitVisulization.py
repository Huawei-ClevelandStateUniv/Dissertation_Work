#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 28 14:59:11 2019

# This file show the plot of the normal gait

@author: huawei
"""

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt


j = 6
s = 1
        
if j < 4:
    store_path = 'normal_walking/F'+str(j)+'_S'+str(s)
else:
    store_path = 'normal_walking/M'+str(j-4)+'_S'+str(s)

save_dof1 = store_path +'/Motion_normalwalking_1_100HZ.txt'
save_mom1 = store_path +'/Moment_normalwalking_1_100HZ.txt'
save_grf1 = store_path +'/GRF_nromalwalking_1_100HZ.txt'
save_dof2 = store_path +'/Motion_normalwalking_2_100HZ.txt'
save_mom2 = store_path +'/Moment_normalwalking_2_100HZ.txt'
save_grf2 = store_path +'/GRF_nromalwalking_2_100HZ.txt'

save_pelvis1 = store_path +'/Pelvis_nromalwalking_1_100HZ.txt'
save_pelvis2 = store_path +'/Pelvis_nromalwalking_2_100HZ.txt'


motion1 = np.loadtxt(save_dof1)
moment1 = np.loadtxt(save_mom1)
grf1 = np.loadtxt(save_grf1)
pelvis1 = np.loadtxt(save_pelvis1)

motion2 = np.loadtxt(save_dof2)
moment2 = np.loadtxt(save_mom2)
grf2 = np.loadtxt(save_grf2)
pelvis2 = np.loadtxt(save_pelvis2)

start_frame = 0
end_frame = 500

time = np.linspace(0, 30, 3000, endpoint=False)

fig1 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 1 Motion')
ax1 = plt.subplot(3, 2, 1)
plt.ylabel('Left Hip (deg)')
ax2 = plt.subplot(3, 2, 2)
plt.ylabel('Right Hip (deg)')
ax3 = plt.subplot(3, 2, 3)
plt.ylabel('Left Knee (deg)')
plt.xlabel('Time (sec)')
ax4 = plt.subplot(3, 2, 4)
plt.ylabel('Right Knee (deg)')
ax5 = plt.subplot(3, 2, 5)
plt.ylabel('Left Ankle (deg)')
ax6 = plt.subplot(3, 2, 6)
plt.ylabel('Right Ankle (deg)')
plt.xlabel('Time (sec)')

ax1.plot(time[start_frame:end_frame], motion1[start_frame:end_frame, 3]*180/pi)
ax3.plot(time[start_frame:end_frame], motion1[start_frame:end_frame, 4]*180/pi)
ax5.plot(time[start_frame:end_frame], motion1[start_frame:end_frame, 5]*180/pi)

ax2.plot(time[start_frame:end_frame], motion1[start_frame:end_frame, 6]*180/pi)
ax4.plot(time[start_frame:end_frame], motion1[start_frame:end_frame, 7]*180/pi)
ax6.plot(time[start_frame:end_frame], motion1[start_frame:end_frame, 8]*180/pi)


fig2 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 1 GRF')
ax21 = plt.subplot(3, 2, 1)
plt.ylabel('Left Fx (deg)')
ax22 = plt.subplot(3, 2, 2)
plt.ylabel('Right Fx (deg)')
ax23 = plt.subplot(3, 2, 3)
plt.ylabel('Left Fy (deg)')
plt.xlabel('Time (sec)')
ax24 = plt.subplot(3, 2, 4)
plt.ylabel('Right Fy (deg)')
ax25 = plt.subplot(3, 2, 5)
plt.ylabel('Left Mx (deg)')
ax26 = plt.subplot(3, 2, 6)
plt.ylabel('Right Mx (deg)')
plt.xlabel('Time (sec)')

ax21.plot(time[start_frame:end_frame], grf1[start_frame:end_frame, 0])
ax23.plot(time[start_frame:end_frame], grf1[start_frame:end_frame, 1])
ax25.plot(time[start_frame:end_frame], grf1[start_frame:end_frame, 2])

ax22.plot(time[start_frame:end_frame], grf1[start_frame:end_frame, 3])
ax24.plot(time[start_frame:end_frame], grf1[start_frame:end_frame, 4])
ax26.plot(time[start_frame:end_frame], grf1[start_frame:end_frame, 5])


fig3 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 2 Moment')
ax31 = plt.subplot(3, 2, 1)
plt.ylabel('Left Hip (Nm)')
ax32 = plt.subplot(3, 2, 2)
plt.ylabel('Right Hip (Nm)')
ax33 = plt.subplot(3, 2, 3)
plt.ylabel('Left Knee (Nm)')
plt.xlabel('Time (sec)')
ax34 = plt.subplot(3, 2, 4)
plt.ylabel('Right Knee (Nm)')
ax35 = plt.subplot(3, 2, 5)
plt.ylabel('Left Ankle (Nm)')
ax36 = plt.subplot(3, 2, 6)
plt.ylabel('Right Ankle (Nm)')
plt.xlabel('Time (sec)')


ax31.plot(time[start_frame:end_frame], moment1[start_frame:end_frame, 0])
ax33.plot(time[start_frame:end_frame], moment1[start_frame:end_frame, 1])
ax35.plot(time[start_frame:end_frame], moment1[start_frame:end_frame, 2])

ax32.plot(time[start_frame:end_frame], moment1[start_frame:end_frame, 3])
ax34.plot(time[start_frame:end_frame], moment1[start_frame:end_frame, 4])
ax36.plot(time[start_frame:end_frame], moment1[start_frame:end_frame, 5])


fig4 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 2 Motion')
ax41 = plt.subplot(3, 2, 1)
plt.ylabel('Left Hip (deg)')
ax42 = plt.subplot(3, 2, 2)
plt.ylabel('Right Hip (deg)')
ax43 = plt.subplot(3, 2, 3)
plt.ylabel('Left Knee (deg)')
plt.xlabel('Time (sec)')
ax44 = plt.subplot(3, 2, 4)
plt.ylabel('Right Knee (deg)')
ax45 = plt.subplot(3, 2, 5)
plt.ylabel('Left Ankle (deg)')
ax46 = plt.subplot(3, 2, 6)
plt.ylabel('Right Ankle (deg)')
plt.xlabel('Time (sec)')

ax41.plot(time[start_frame:end_frame], motion2[start_frame:end_frame, 3])
ax43.plot(time[start_frame:end_frame], motion2[start_frame:end_frame, 4])
ax45.plot(time[start_frame:end_frame], motion2[start_frame:end_frame, 5])

ax42.plot(time[start_frame:end_frame], motion2[start_frame:end_frame, 6])
ax44.plot(time[start_frame:end_frame], motion2[start_frame:end_frame, 7])
ax46.plot(time[start_frame:end_frame], motion2[start_frame:end_frame, 8])


fig5 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 2 GRF')
ax51 = plt.subplot(3, 2, 1)
plt.ylabel('Left Fx (deg)')
ax52 = plt.subplot(3, 2, 2)
plt.ylabel('Right Fx (deg)')
ax53 = plt.subplot(3, 2, 3)
plt.ylabel('Left Fy (deg)')
plt.xlabel('Time (sec)')
ax54 = plt.subplot(3, 2, 4)
plt.ylabel('Right Fy (deg)')
ax55 = plt.subplot(3, 2, 5)
plt.ylabel('Left Mx (deg)')
ax56 = plt.subplot(3, 2, 6)
plt.ylabel('Right Mx (deg)')
plt.xlabel('Time (sec)')

ax51.plot(time[start_frame:end_frame], grf2[start_frame:end_frame, 0])
ax53.plot(time[start_frame:end_frame], grf2[start_frame:end_frame, 1])
ax55.plot(time[start_frame:end_frame], grf2[start_frame:end_frame, 2])

ax52.plot(time[start_frame:end_frame], grf2[start_frame:end_frame, 3])
ax54.plot(time[start_frame:end_frame], grf2[start_frame:end_frame, 4])
ax56.plot(time[start_frame:end_frame], grf2[start_frame:end_frame, 5])

fig6 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 2 Moment')
ax61 = plt.subplot(3, 2, 1)
plt.ylabel('Left Hip (Nm)')
ax62 = plt.subplot(3, 2, 2)
plt.ylabel('Right Hip (Nm)')
ax63 = plt.subplot(3, 2, 3)
plt.ylabel('Left Knee (Nm)')
plt.xlabel('Time (sec)')
ax64 = plt.subplot(3, 2, 4)
plt.ylabel('Right Knee (Nm)')
ax65 = plt.subplot(3, 2, 5)
plt.ylabel('Left Ankle (Nm)')
ax66 = plt.subplot(3, 2, 6)
plt.ylabel('Right Ankle (Nm)')
plt.xlabel('Time (sec)')

ax61.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 0])
ax63.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 1])
ax65.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 2])

ax62.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 3])
ax64.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 4])
ax66.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 5])


fig7 = plt.figure(figsize=(12, 12))
plt.title('Normal Walking Period 2 GRF')
ax71 = plt.subplot(3, 2, 1)
plt.ylabel('PelvisX (N)')
ax72 = plt.subplot(3, 2, 2)
plt.ylabel('PelvisForwardPitch (Nm)')
ax73 = plt.subplot(3, 2, 3)
plt.ylabel('PelvisY (N)')
plt.xlabel('Time (sec)')
ax74 = plt.subplot(3, 2, 4)
plt.ylabel('PelvisRightRoll (Nm)')
ax75 = plt.subplot(3, 2, 5)
plt.ylabel('PelvisZ (N)')
ax76 = plt.subplot(3, 2, 6)
plt.ylabel('TrunkFlexion (Nm)')
plt.xlabel('Time (sec)')

ax71.plot(time[start_frame:end_frame], pelvis1[start_frame:end_frame, 0])
ax73.plot(time[start_frame:end_frame], pelvis1[start_frame:end_frame, 1])
ax75.plot(time[start_frame:end_frame], pelvis1[start_frame:end_frame, 2])

ax72.plot(time[start_frame:end_frame], pelvis1[start_frame:end_frame, 4])
ax74.plot(time[start_frame:end_frame], pelvis1[start_frame:end_frame, 5])
ax76.plot(time[start_frame:end_frame], pelvis1[start_frame:end_frame, 6])

#fig6 = plt.figure(figsize=(12, 12))
#plt.title('Normal Walking Period 2 Moment')
#ax61 = fig6.add_subplot(3, 2, 1)
#plt.ylabel('Left Hip (Nm)')
#ax62 = fig6.add_subplot(3, 2, 2)
#plt.ylabel('Right Hip (Nm)')
#ax63 = fig6.add_subplot(3, 2, 3)
#plt.ylabel('Left Knee (Nm)')
#plt.xlabel('Time (sec)')
#ax64 = fig6.add_subplot(3, 2, 4)
#plt.ylabel('Right Knee (Nm)')
#ax65 = fig6.add_subplot(3, 2, 5)
#plt.ylabel('Left Ankle (Nm)')
#ax66 = fig6.add_subplot(3, 2, 6)
#plt.ylabel('Right Ankle (Nm)')
#plt.xlabel('Time (sec)')
#
#ax61.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 0])
#ax63.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 1])
#ax65.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 2])
#
#ax62.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 3])
#ax64.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 4])
#ax66.plot(time[start_frame:end_frame], moment2[start_frame:end_frame, 5])