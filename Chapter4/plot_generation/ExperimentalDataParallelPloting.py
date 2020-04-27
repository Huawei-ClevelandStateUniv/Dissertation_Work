#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 17:18:03 2018

4D parallel plot of experiment data

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt

import matplotlib.ticker as ticker

def parallel_coordinates(data_sets, style=None, Label = None, SFont=14):

    dims = len(data_sets[0])
    x    = range(dims)
    fig, axes = plt.subplots(1, dims-1, sharey=False, figsize=(6, 4))

    if style is None:
        style = ['y-']*len(data_sets)

    # Calculate the limits on the data
    min_max_range = list()
    for m in zip(*data_sets):
        mn = min(m)
        mx = max(m)
        if mn == mx:
            mn -= 0.5
            mx = mn + 1.
        r  = float(mx - mn)
        min_max_range.append((mn, mx, r))

    # Normalize the data sets
    norm_data_sets = list()
    for ds in data_sets:
        nds = [(value - min_max_range[dimension][0]) / 
                min_max_range[dimension][2] 
                for dimension,value in enumerate(ds)]
        norm_data_sets.append(nds)
    data_sets = norm_data_sets

    # Plot the datasets on all the subplots
    for i, ax in enumerate(axes):
        for dsi, d in enumerate(data_sets):
            ax.plot(x, d, style[dsi])
        ax.set_xlim([x[i], x[i+1]])

    # Set the x axis ticks 
    for dimension, (axx,xx) in enumerate(zip(axes, x[:-1])):
        axx.xaxis.set_major_locator(ticker.FixedLocator([xx]))
        ticks = len(axx.get_yticklabels())
        labels = list()
        step = min_max_range[dimension][2] / (ticks - 3)
        mn   = min_max_range[dimension][0] - step
        for i in range(ticks):
            v = mn + i*step
            labels.append('%4.2f' % v)
        axx.set_yticklabels(labels)
        #axx.patch.set_visible(False)
        #axx.axis('off')


    # Move the final axis' ticks to the right-hand side
    axx = plt.twinx(axes[-1])
    dimension += 1
    axx.xaxis.set_major_locator(ticker.FixedLocator([x[-2], x[-1]]))
    ticks = len(axx.get_yticklabels())
    step = min_max_range[dimension][2] / (ticks - 1)
    mn   = min_max_range[dimension][0]
    labels = ['%4.2f' % (mn + i*step) for i in range(ticks)]
    axx.set_yticklabels(labels)


    # Stack the subplots 
    plt.subplots_adjust(wspace=0)

    return plt

if __name__ == '__main__':
    
    Subj = 5
    Trail = 2
    
    traj_full = np.loadtxt('Data/2DoF Data Simplify/Subj05/JointsGFL000' + str(Trail+1) + '.txt')
    
    DataVec = (traj_full[8000:15000, 1:5])
    colors = ['C1-'] * len(DataVec[:, 0])
    
    Max = np.max(DataVec, axis = 0)
    Min = np.min(DataVec, axis = 0)
#    
#    num_batch = 11
#    
#    Bt1 = np.linspace(Min[0], Max[0], num_batch, endpoint=True)
#    Bt2 = np.linspace(Min[1], Max[1], num_batch, endpoint=True)
#    Bt3 = np.linspace(Min[2], Max[2], num_batch, endpoint=True)
#    Bt4 = np.linspace(Min[3], Max[3], num_batch, endpoint=True)
#    
#    for k in range(11):
#        for j in range(11):
#            for p in range(11):
#                for q in range(11):
#                    DataVec = np.vstack((DataVec, np.array([Bt1[k], Bt2[j], Bt3[p], Bt4[q]])))
#    
#
#    colors.extend(['r'] *11*11*11*11)
#    
    parallel_coordinates(DataVec, style = colors).show()
    
#    SFont = 12
#    width = 0.25
#    
#    index = np.array([0, 1, 2, 3])
#    value= np.array([0, 0, 0, 0])
#
#    fig2 = plt.figure(figsize=(6, 4))
#    plt.xticks(index+0.15, label, fontsize=SFont)
#    plt.yticks(fontsize=SFont)
#    plt.ylabel('Degree (Degree/s)', fontsize=SFont)
#    plt.bar(index, value, width)