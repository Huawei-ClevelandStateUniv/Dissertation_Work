# -*- coding: utf-8 -*-
"""
Created on Mon Aug  6 22:30:10 2018

@author: Huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from normalized_polynomial import polynomial_value
import matplotlib


def Obj(x):
    
    Obj = 0
    
    for k in range(len(Swingt[:, 0])):
        
        L0 = Swingy[k, 0]
        L1 = Swingy[k, -1]
        T = Swingt[k, -1]
        
        Fit = polynomial_value(L0, L1, T, Swingt[k, :], x)
        
        Diff_Swingx = Swingy[k, :] - Fit

        Disp_end = (Swingy[k, -1] - Fit[-1])**2
        
        Velo_start = ((Swingy[k, 1]-Swingy[k, 0])/(Swingt[k, 1]-Swingt[k, 0]) 
                    - (Fit[1]-Fit[0])/(Swingt[k, 1]-Swingt[k, 0]))**2
        
        Velo_end = ((Swingy[k, -1]-Swingy[k, -2])/(Swingt[k, -1]-Swingt[k, -2]) 
                    - (Fit[-1]-Fit[-2])/(Swingt[k, -1]-Swingt[k, -2]))**2

        Obj = Obj + np.sum(Diff_Swingx**2) + (500*Velo_start + 1000*Disp_end 
                  + 500*Velo_end)
        
    Obj = Obj/len(Swingt[:, 0])
    
    return Obj


def get_swingy_polynomial(Lswingy, Rswingy, Lswingt, Rswingt, num_par,
                           repeat, store_path):

    LEd = len(Lswingy[:, 0])
    REd = len(Rswingy[:, 0])
    
    fig = plt.figure(figsize=(6, 6))
    for k in range(LEd):
        plt.plot(Lswingt[k, :], Lswingy[k, :], 'r-')
    for j in range(REd):
        plt.plot(Rswingt[j, :], Rswingy[j, :], 'b-')
        
    fig.savefig(store_path + '/Path_Swingy_plot.png')
    
    Ed = LEd + REd
    global Swingy
    global Swingt
    Swingy = np.vstack((Lswingy, Rswingy))
    Swingt = np.vstack((Lswingt, Rswingt))
    
    Result = np.zeros((repeat, num_par))
    for m in range(repeat):
        np.random.seed()
        x0 = 10 - 20*np.random.random(num_par)
        
        res = minimize(Obj, x0, method='BFGS', options={'disp': True})
        
        Result[m, :] = res['x']
    
    Coef = np.average(Result, axis=0)
    Coef_st = np.std(Result, axis=0)


    RMS = np.zeros(Ed)
    for k in range(Ed):
    
        L0 = Swingy[k, 0]
        L1 = Swingy[k, -1]
        T = Swingt[k, -1]
        
        Fit = polynomial_value(L0, L1, T, Swingt[k, :], Coef)
        
        RMS[k] = np.mean(np.square(Swingy[k, :] - Fit))

    with open(store_path + '/RMS_Swingy_normalized'+str(num_par)+'.txt', 'w') as outfile:
        StringP = ""
        for i in range(len(RMS)):
            StringP += str(RMS[i])
            StringP += "\n"
        outfile.write(StringP)
        
    with open(store_path + '/Coe_Swingy_normalized'+str(num_par)+'.txt', 'w') as outfile:
        StringP = ""
        for i in range(len(Result[:, 0])):
            for j in range(len(Result[0, :])):
                StringP += str(Result[i, j])
                StringP += " "
            StringP += "\n"
        outfile.write(StringP)
        
    with open(store_path + '/Coe_Swingy_normalized'+str(num_par)+'_std.txt', 'w') as outfile:
        StringP = ""
        for j in range(len(Result[0, :])):
            StringP += str(Coef[j])
            StringP += " "
            StringP += str(Coef_st[j])
            StringP += "\n"
        outfile.write(StringP)


    index_max = RMS.argmax()
    index_min = RMS.argmin()
    index_med = np.argsort(RMS)[len(RMS)//2]
    
    L0 = Swingy[index_min, 0]
    L1 = Swingy[index_min, -1]
    T = Swingt[index_min, -1]
    
    L01 = Swingy[index_med, 0]
    L11 = Swingy[index_med, -1]
    T1 = Swingt[index_med, -1]
    
    L02 = Swingy[index_max, 0]
    L12 = Swingy[index_max, -1]
    T2 = Swingt[index_max, -1]
    
    Fit = polynomial_value(L0, L1, T, Swingt[index_min, :], Coef)
    
    Fit1 = polynomial_value(L01, L11, T1, Swingt[index_med, :], Coef)
    
    Fit2 = polynomial_value(L02, L12, T2, Swingt[index_max, :], Coef)
    
    fig1 = plt.figure(figsize=(12, 8))
    
    matplotlib.rc('xtick', labelsize=16) 
    matplotlib.rc('ytick', labelsize=16)
    
    ax1 = fig1.add_subplot(1,3,1)
    plt.xlabel('t (s)', fontsize=16)
    plt.ylabel('Disp (m)', fontsize=16)
    plt.title('Best Fit', fontsize=16)
    ax1.plot(Swingt[index_min, :], Swingy[index_min, :], 'r-')
    ax1.plot(Swingt[index_min, :], Fit, 'g.')
    
    ax2 = fig1.add_subplot(1,3,2)
    plt.xlabel('t (s)', fontsize=16)
    plt.title('Median Fit', fontsize=16)
    ax2.plot(Swingt[index_med, :], Swingy[index_med, :], 'r-')
    ax2.plot(Swingt[index_med, :], Fit1, 'g.')
    
    ax3 = fig1.add_subplot(1,3,3)
    plt.xlabel('t (s)', fontsize=16)
    plt.title('Worst Fit', fontsize=16)
    ax3.plot(Swingt[index_max, :], Swingy[index_max, :], 'r-', label = 'experiment data')
    ax3.plot(Swingt[index_max, :], Fit2, 'g.', label = 'polynomial fit')
    plt.legend(fontsize=13)
    
    fig1.savefig(store_path + '/Fit_Swingy_info.png')