# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 21:14:26 2018

@author: Huawei
"""

import numpy as np
import matplotlib.pyplot as plt

# Colum names

# 0 : time column

# 2 : GRF Left x
# 3 : GRF Left y
# 4 : GRF Left moment  

# 5 : GRF Right x
# 6 : GRF Right y
# 7 : GRF Right moment 

# 8 : Pelvis x
# 9 : Pelvis y 

# 10 : Hip x
# 11 : Hip y

# 12 : Left Ankle x
# 13 : Left Ankle y 

# 14 : Right Ankle x
# 15 : Right Ankle y

# 16 : Left Heel x
# 17 : Left Heel y

# 18 : Right Heel x
# 19 : Right Heel y

# 16 : Left Knee x
# 17 : Left Knee y

# 18 : Right Knee x
# 19 : Right Knee y

St = 0
Ed = 73

Len = 100

SwingAnkx = np.zeros((2*(Ed-St), Len))
SwingAnky = np.zeros((2*(Ed-St), Len))

SwingHeelx = np.zeros((2*(Ed-St), Len))
SwingHeely = np.zeros((2*(Ed-St), Len))

SwingKneex = np.zeros((2*(Ed-St), Len))
SwingKneey = np.zeros((2*(Ed-St), Len))

StanceKneex = np.zeros((2*(Ed-St), Len))
StanceKneey = np.zeros((2*(Ed-St), Len))

StanceHipx = np.zeros((2*(Ed-St), Len))
StanceHipy = np.zeros((2*(Ed-St), Len))

StanceHeelx = np.zeros((2*(Ed-St), Len))
StanceHeely = np.zeros((2*(Ed-St), Len))

Swingt = np.zeros((2*(Ed-St), Len))
Stancet = np.zeros((2*(Ed-St), Len))

for k in range(St, Ed):
    #stance = np.loadtxt('SepData/StanceMotion0.txt')
    Lswing = np.loadtxt('SepData_Force/SwingMotion'+str(k)+'L.txt')
    Lstance = np.loadtxt('SepData_Force/StanceMotion'+str(k)+'L.txt')
    
    Rswing = np.loadtxt('SepData_Force/SwingMotion'+str(k)+'R.txt')
    Rstance = np.loadtxt('SepData_Force/StanceMotion'+str(k)+'R.txt')
    
    LtimeSwO = np.linspace(0, len(Lswing[:, 0])*0.01, len(Lswing[:, 0]))
    LtimeStO = np.linspace(0, len(Lstance[:, 0])*0.01, len(Lstance[:, 0]))
    
    RtimeSwO = np.linspace(0, len(Rswing[:, 0])*0.01, len(Rswing[:, 0]))
    RtimeStO = np.linspace(0, len(Rstance[:, 0])*0.01, len(Rstance[:, 0]))
    
    LtimeSw = np.linspace(0, len(Lswing[:, 0])*0.01, Len)
    LtimeSt = np.linspace(0, len(Lstance[:, 0])*0.01, Len)
    
    RtimeSw = np.linspace(0, len(Rswing[:, 0])*0.01, Len)
    RtimeSt = np.linspace(0, len(Rstance[:, 0])*0.01, Len)
    
    SwingLAnkx = (Lswing[:, 12] - Lswing[:, 10])
    SwingRAnkx = (Rswing[:, 14] - Rswing[:, 10])
    
    SwingLAnky = (Lswing[:, 13] - Lswing[:, 11])
    SwingRAnky = (Rswing[:, 15] - Rswing[:, 11])
    
    SwingLKneex = (Lswing[:, 12] - Lswing[:, 20])
    SwingRKneex = (Rswing[:, 14] - Rswing[:, 22])
    
    SwingLKneey = (Lswing[:, 13] - Lswing[:, 21])
    SwingRKneey = (Rswing[:, 15] - Rswing[:, 23])
    
    SwingLHeelx = -(Lswing[:, 12] - Lswing[:, 16])
    SwingRHeelx = -(Rswing[:, 14] - Rswing[:, 18])
    
    SwingLHeely = -(Lswing[:, 13] - Lswing[:, 17])
    SwingRHeely = -(Rswing[:, 15] - Rswing[:, 19])
    
    StanceLHipx = -(Lstance[:, 12] - Lstance[:, 10])
    StanceRHipx = -(Rstance[:, 14] - Rstance[:, 10])
    
    StanceLHipy = -(Lstance[:, 13] - Lstance[:, 11])
    StanceRHipy = -(Rstance[:, 15] - Rstance[:, 11])
    
    StanceLHeelx = (Lstance[:, 12] - Lstance[:, 16])
    StanceRHeelx = (Rstance[:, 14] - Rstance[:, 18])
    
    StanceLHeely = (Lstance[:, 13] - Lstance[:, 17])
    StanceRHeely = (Rstance[:, 15] - Rstance[:, 19])
    
    StanceLKneex = -(Lstance[:, 12] - Lstance[:, 20])
    StanceRKneex = -(Rstance[:, 14] - Rstance[:, 22])
    
    StanceLKneey = -(Lstance[:, 13] - Lstance[:, 21])
    StanceRKneey = -(Rstance[:, 15] - Rstance[:, 23])
    
    SwingAnkx[k, :] = np.interp(LtimeSw, LtimeSwO, SwingLAnkx)
    SwingAnkx[k+(Ed-St), :] = np.interp(RtimeSw, RtimeSwO, SwingRAnkx)
    
    SwingAnky[k, :] = np.interp(LtimeSw, LtimeSwO, SwingLAnky)
    SwingAnky[k+(Ed-St), :] = np.interp(RtimeSw, RtimeSwO, SwingRAnky)
    
    SwingHeelx[k, :] = np.interp(LtimeSw, LtimeSwO, SwingLHeelx)
    SwingHeelx[k+(Ed-St), :] = np.interp(RtimeSw, RtimeSwO, SwingRHeelx)
    
    SwingHeely[k, :] = np.interp(LtimeSw, LtimeSwO, SwingLHeely)
    SwingHeely[k+(Ed-St), :] = np.interp(RtimeSw, RtimeSwO, SwingRHeely)
    
    SwingKneex[k, :] = np.interp(LtimeSw, LtimeSwO, SwingLKneex)
    SwingKneex[k+(Ed-St), :] = np.interp(RtimeSw, RtimeSwO, SwingRKneex)
    
    SwingKneey[k, :] = np.interp(LtimeSw, LtimeSwO, SwingLKneey)
    SwingKneey[k+(Ed-St), :] = np.interp(RtimeSw, RtimeSwO, SwingRKneey)
    
    Swingt[k, :] = LtimeSw
    Swingt[k+(Ed-St), :] = RtimeSw
    
    StanceHipx[k, :] = np.interp(LtimeSt, LtimeStO, StanceLHipx)
    StanceHipx[k+(Ed-St), :] = np.interp(RtimeSt, RtimeStO, StanceRHipx)
    
    StanceHipy[k, :] = np.interp(LtimeSt, LtimeStO, StanceLHipy)
    StanceHipy[k+(Ed-St), :] = np.interp(RtimeSt, RtimeStO, StanceRHipy)
    
    StanceHeelx[k, :] = np.interp(LtimeSt, LtimeStO, StanceLHeelx)
    StanceHeelx[k+(Ed-St), :] = np.interp(RtimeSt, RtimeStO, StanceRHeelx)
    
    StanceHeely[k, :] = np.interp(LtimeSt, LtimeStO, StanceLHeely)
    StanceHeely[k+(Ed-St), :] = np.interp(RtimeSt, RtimeStO, StanceRHeely)
    
    StanceKneex[k, :] = np.interp(LtimeSt, LtimeStO, StanceLKneex)
    StanceKneex[k+(Ed-St), :] = np.interp(RtimeSt, RtimeStO, StanceRKneex)
    
    StanceKneey[k, :] = np.interp(LtimeSt, LtimeStO, StanceLHeely)
    StanceKneey[k+(Ed-St), :] = np.interp(RtimeSt, RtimeStO, StanceRKneey)
    
    Stancet[k, :] = LtimeSt
    Stancet[k+(Ed-St), :] = RtimeSt
    
fig2 = plt.figure(figsize=(18,18))
ax1 = fig2.add_subplot(4,2,1)
ax2 = fig2.add_subplot(4,2,2)
ax3 = fig2.add_subplot(4,2,3)
ax4 = fig2.add_subplot(4,2,4)

ax5 = fig2.add_subplot(4,2,5)
ax6 = fig2.add_subplot(4,2,6)
ax7 = fig2.add_subplot(4,2,7)
ax8 = fig2.add_subplot(4,2,8)

for m in range(2*(Ed-St)):
    ax1.plot(Swingt[m, :], SwingAnkx[m, :], '-')
    
    ax2.plot(Swingt[m, :], SwingAnky[m, :], '-')

    ax3.plot(Stancet[m, :], StanceHipx[m, :], '-')
    
    ax4.plot(Stancet[m, :], StanceHipy[m, :], '-')
    
    ax5.plot(Swingt[m, :], SwingHeelx[m, :], '-')

    ax6.plot(Swingt[m, :], SwingHeely[m, :], '-')
    
    ax7.plot(Stancet[m, :], StanceHeelx[m, :], '-')

    ax8.plot(Stancet[m, :], StanceHeely[m, :], '-')
    
fig3 = plt.figure(figsize=(18,18))
ax9 = fig3.add_subplot(4,2,1)
ax10 = fig3.add_subplot(4,2,2)
ax11 = fig3.add_subplot(4,2,3)
ax12 = fig3.add_subplot(4,2,4)

ax13 = fig3.add_subplot(4,2,5)
ax14 = fig3.add_subplot(4,2,6)
ax15 = fig3.add_subplot(4,2,7)
ax16 = fig3.add_subplot(4,2,8)

for m in range(2*(Ed-St)):
    ax9.plot( SwingAnkx[m, :], '-')
    
    ax10.plot( SwingAnky[m, :], '-')

    ax11.plot( StanceHipx[m, :], '-')
    
    ax12.plot( StanceHipy[m, :], '-')
    
    ax13.plot( SwingHeelx[m, :], '-')

    ax14.plot( SwingHeely[m, :], '-')
    
    ax15.plot( StanceHeelx[m, :], '-')

    ax16.plot( StanceHeely[m, :], '-')

with open('SepData_Force_For/SwingAnkxF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(SwingAnkx[:, 0])):
        for j in range(len(SwingAnkx[0, :])):
            StringP1 += str(SwingAnkx[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/SwingAnkyF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(SwingAnky[:, 0])):
        for j in range(len(SwingAnky[0, :])):
            StringP1 += str(SwingAnky[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)

with open('SepData_Force_For/StanceHipxF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(StanceHipx[:, 0])):
        for j in range(len(StanceHipx[0, :])):
            StringP1 += str(StanceHipx[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/StanceHipyF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(StanceHipy[:, 0])):
        for j in range(len(StanceHipy[0, :])):
            StringP1 += str(StanceHipy[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/SwingKneexF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(SwingKneex[:, 0])):
        for j in range(len(SwingKneex[0, :])):
            StringP1 += str(SwingKneex[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/SwingKneeyF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(SwingKneey[:, 0])):
        for j in range(len(SwingKneey[0, :])):
            StringP1 += str(SwingKneey[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)

with open('SepData_Force_For/StanceKneexF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(StanceKneex[:, 0])):
        for j in range(len(StanceKneex[0, :])):
            StringP1 += str(StanceKneex[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/StanceKneeyF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(StanceKneey[:, 0])):
        for j in range(len(StanceKneey[0, :])):
            StringP1 += str(StanceKneey[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/SwingHeelxF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(SwingHeelx[:, 0])):
        for j in range(len(SwingHeelx[0, :])):
            StringP1 += str(SwingHeelx[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/SwingHeelyF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(SwingHeely[:, 0])):
        for j in range(len(SwingHeely[0, :])):
            StringP1 += str(SwingHeely[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)

with open('SepData_Force_For/StanceHeelxF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(StanceHeelx[:, 0])):
        for j in range(len(StanceHeelx[0, :])):
            StringP1 += str(StanceHeelx[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/StanceHeelyF.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(StanceHeely[:, 0])):
        for j in range(len(StanceHeely[0, :])):
            StringP1 += str(StanceHeely[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
#    
with open('SepData_Force_For/Swingt.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(Swingt[:, 0])):
        for j in range(len(Swingt[0, :])):
            StringP1 += str(Swingt[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)
    
with open('SepData_Force_For/Stancet.txt', 'w') as outfile1:
    StringP1 = ""
    for i in range(len(Stancet[:, 0])):
        for j in range(len(Stancet[0, :])):
            StringP1 += str(Stancet[i, j])
            StringP1 += " "
        StringP1 += "\n"
    outfile1.write(StringP1)