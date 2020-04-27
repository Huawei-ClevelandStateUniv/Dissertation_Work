#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 16 11:25:36 2018

Eigenvalue percentage plot of all controller types.

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt

Subj = 5   # subject number
Trial = 2  # experimental trial number

# load experimental data
X_exp = np.loadtxt('../experimental_data/Subj05/JointsGFL000' + str(Trial+1) + '.txt')

Std = np.std(X_exp[8000:15000, 1:5], axis = 0)  # standard deviation of experimental data

StdMean = (Std[0] + Std[1])/2*3.1416/180  # average variation

# load calculated eigenvalue percentage and forward simulation percentage
Eig_PD0 = np.loadtxt('../stability_check/PD/Analysis/Eps0/EigPercentage.txt' )
Eig_PD2 = np.loadtxt('../stability_check/PD/Analysis/Eps2/EigPercentage.txt' )
Eig_PD3 = np.loadtxt('../stability_check/PD/Analysis/Eps3/EigPercentage.txt' )
Eig_PD4 = np.loadtxt('../stability_check/PD/Analysis/Eps4/EigPercentage.txt' )

RMS_PD0 = np.loadtxt('../stability_check/PD/Analysis/Eps0RMSAll.txt' )
RMS_PD2 = np.loadtxt('../stability_check/PD/Analysis/Eps2RMSAll.txt' )
RMS_PD3 = np.loadtxt('../stability_check/PD/Analysis/Eps3RMSAll.txt' )
RMS_PD4 = np.loadtxt('../stability_check/PD/Analysis/Eps4RMSAll.txt' )

Eig_FPD0 = np.loadtxt('../stability_check/FPD/Analysis/Eps0/EigPercentage.txt' )
Eig_FPD2 = np.loadtxt('../stability_check/FPD/Analysis/Eps2/EigPercentage.txt' )
Eig_FPD3 = np.loadtxt('../stability_check/FPD/Analysis/Eps3/EigPercentage.txt' )
Eig_FPD4 = np.loadtxt('../stability_check/FPD/Analysis/Eps4/EigPercentage.txt' )

RMS_FPD0 = np.loadtxt('../stability_check/FPD/Analysis/Eps0RMSAll.txt' )
RMS_FPD2 = np.loadtxt('../stability_check/FPD/Analysis/Eps2RMSAll.txt' )
RMS_FPD3 = np.loadtxt('../stability_check/FPD/Analysis/Eps3RMSAll.txt' )
RMS_FPD4 = np.loadtxt('../stability_check/FPD/Analysis/Eps4RMSAll.txt' )

Eig_NN0 = np.loadtxt('../stability_check/NN14/Analysis/Eps0/EigPercentage.txt' )
Eig_NN6 = np.loadtxt('../stability_check/NN14/Analysis/Eps6/EigPercentage.txt' )
Eig_NN8 = np.loadtxt('../stability_check/NN14/Analysis/Eps8/EigPercentage.txt' )
Eig_NN10 = np.loadtxt('../stability_check/NN14/Analysis/Eps10/EigPercentage.txt' )

RMS_NN0 = np.loadtxt('../stability_check/NN14/Analysis/Eps0RMSAll.txt' )
RMS_NN6 = np.loadtxt('../stability_check/NN14/Analysis/Eps6RMSAll.txt' )
RMS_NN8 = np.loadtxt('../stability_check/NN14/Analysis/Eps8RMSAll.txt' )
RMS_NN10 = np.loadtxt('../stability_check/NN14/Analysis/Eps10RMSAll.txt' )


def StableCount(RMS):
    """
    A function to check the forward simulation stability based on the variation range
    """    
    Sum = 0
    for k in range(len(RMS[:, 0])):
        for j in range(len(RMS[0, :])):
            if RMS[k, j] <= 3*StdMean:  # variation of the forward simulation needs
                                        # be inside 3 times of the experimental data variation
                Sum += 1
                
    return Sum

# initialize the percentage vector
top_PD = np.zeros(4)
top_FPD = np.zeros(4)
top_NN = np.zeros(4)

RMS_PD = np.zeros(4)
RMS_FPD = np.zeros(4)
RMS_NN = np.zeros(4)

# calculate the percentage
top_PD[0] = np.mean(Eig_PD0)
top_PD[1] = np.mean(Eig_PD2)
top_PD[2] = np.mean(Eig_PD3)
top_PD[3] = np.mean(Eig_PD4)

RMS_PD[0] = StableCount(RMS_PD0)/(11*11*11*11)*100
RMS_PD[1] = StableCount(RMS_PD2)/(11*11*11*11)*100
RMS_PD[2] = StableCount(RMS_PD3)/(11*11*11*11)*100
RMS_PD[3] = StableCount(RMS_PD4)/(11*11*11*11)*100

top_FPD[0] = np.mean(Eig_FPD0)
top_FPD[1] = np.mean(Eig_FPD2)
top_FPD[2] = np.mean(Eig_FPD3)
top_FPD[3] = np.mean(Eig_FPD4)

RMS_FPD[0] = StableCount(RMS_FPD0)/(11*11*11*11)*100
RMS_FPD[1] = StableCount(RMS_FPD2)/(11*11*11*11)*100
RMS_FPD[2] = StableCount(RMS_FPD3)/(11*11*11*11)*100
RMS_FPD[3] = StableCount(RMS_FPD4)/(11*11*11*11)*100

top_NN[0] = np.mean(Eig_NN0)
top_NN[1] = np.mean(Eig_NN6)
top_NN[2] = np.mean(Eig_NN8)
top_NN[3] = np.mean(Eig_NN10)

RMS_NN[0] = StableCount(RMS_NN0)/(11*11*11*11)*100
RMS_NN[1] = StableCount(RMS_NN6)/(11*11*11*11)*100
RMS_NN[2] = StableCount(RMS_NN8)/(11*11*11*11)*100
RMS_NN[3] = StableCount(RMS_NN10)/(11*11*11*11)*100

# plot the stable percentage bar plot
index = np.arange(4)

bottom = np.zeros_like(index)
width = 0.25

SFont = 14

LABELL = ["DET", "STO 2", "STO 3", "STO 4"]
LABELN = ["DET", "STO 6", "STO 8", "STO 10"]

fig = plt.figure(figsize=(9, 14))

ax11 = fig.add_subplot(3,1,1)
plt.xticks(index+0.15, LABELL, fontsize=SFont)
plt.yticks(fontsize=SFont)
plt.ylabel('% stable trials', fontsize=SFont)
plt.title('PD', fontsize=SFont)
ax12 = fig.add_subplot(3,1,2)
plt.xticks(index+0.15, LABELL, fontsize=SFont)
plt.yticks(fontsize=SFont)
plt.ylabel('% stable trials', fontsize=SFont)
plt.title('FPD', fontsize=SFont)
ax13 = fig.add_subplot(3,1,3)
plt.xticks(index+0.15, LABELN, fontsize=SFont)
plt.yticks(fontsize=SFont)
plt.ylabel('% stable trials', fontsize=SFont)
plt.title('NN14', fontsize=SFont)

ax11.bar(index, top_PD, width, label='Stable Eigenvalues')
ax11.bar(index+0.3, RMS_PD, width, label='Stable Simulations')
ax11.set_ylim(0, 100)

ax12.bar(index, top_FPD, width, label='Stable Eigenvalues')
ax12.bar(index+0.3, RMS_FPD, width, label='Stable Simulations')
ax12.set_ylim(0, 100)

ax13.bar(index, top_NN, width, label='Stable Eigenvalues')
ax13.bar(index+0.3, RMS_NN, width, label='Stable Simulations')
ax13.set_ylim(0, 100)
plt.legend(fontsize=SFont)

