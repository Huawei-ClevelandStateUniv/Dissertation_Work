# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 18:39:41 2018

@author: Huawei
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from hip_ankle_points_calculation import get_hip_point, get_ankle_point


def sep_gait_by_force(motion, grf, st_walking, ed_walking, num_nodes, ext_nodes, ext_gait, delta, const_dicts, plot_sign=False,
                   write_sign=False, store_path=''):
    
    every_node = int(delta/0.01)

    b, a = signal.butter(2, 2.0*8/(50), 'low', analog=False)
    LFy = signal.filtfilt(b, a, grf[:, 1])
    RFy = signal.filtfilt(b, a, grf[:, 7])
    
    Lthigh = const_dicts['ThighLen']
    Lshank = const_dicts['ShankLen']
    
    signL = np.zeros(num_nodes)
    signR = np.zeros(num_nodes)
    
    for indL in range(len(LFy)):
        if LFy[indL] > 100:
            signL[indL] = 1
            
    for indR in range(len(RFy)):
        if RFy[indR] > 100:
            signR[indR] = 1
        
    DsignL = np.diff(signL)
    DsignR = np.diff(signR)
    
    Lhs = (np.where(DsignL==1)[0]).astype(int)
    Lto = (np.where(DsignL==-1)[0]).astype(int)
    
    Rhs = (np.where(DsignR==1)[0]).astype(int)
    Rto = (np.where(DsignR==-1)[0]).astype(int)
    
    Lgait = np.min([len(Lhs), len(Lto)])
    Rgait = np.min([len(Rhs), len(Rto)])
    
    Lswingx = np.zeros((Lgait, 100))
    Lswingy = np.zeros((Lgait, 100))
    
    Lswingt = np.zeros((Lgait, 100))
    
    Lstancex = np.zeros((Lgait, 100))
    Lstancey = np.zeros((Lgait, 100))
    
    Lstancet = np.zeros((Lgait, 100))
    
    Rswingx = np.zeros((Rgait, 100))
    Rswingy = np.zeros((Rgait, 100))
    
    Rswingt = np.zeros((Rgait, 100))
    
    Rstancex = np.zeros((Rgait, 100))
    Rstancey = np.zeros((Rgait, 100))
    
    Rstancet = np.zeros((Rgait, 100))
    
    InitialDataL = np.zeros((num_nodes, 9))
    InitialDataR = np.zeros((num_nodes, 9))
    
    if Lto[0] < Lhs[0]:
        for k in range(Lgait-1):
            for j in range(Lto[k], Lhs[k]):
                InitialDataL[j, 0] = 0
                InitialDataL[j, 1] = (j-Lto[k])*delta
                InitialDataL[j, 2] = (Lhs[k]-Lto[k])*delta
                InitialDataL[j, 3:5] = get_ankle_point(motion[Lto[k], 2], motion[Lto[k], 3], motion[Lto[k], 4], Lthigh, Lshank)
                InitialDataL[j, 5:7] = get_ankle_point(motion[j, 2], motion[j, 3], motion[j, 4], Lthigh, Lshank)
                InitialDataL[j, 7:9] = get_ankle_point(motion[Lhs[k], 2], motion[Lhs[k], 3], motion[Lhs[k], 4], Lthigh, Lshank)
            for i in range(Lhs[k], Lto[k+1]):
                InitialDataL[i, 0] = 1
                InitialDataL[i, 1] = (i-Lhs[k])*delta
                InitialDataL[i, 2] = (Lto[k+1]-Lhs[k])*delta
                InitialDataL[i, 3:5] = get_hip_point(motion[Lhs[k], 2], motion[Lhs[k], 3], motion[Lhs[k], 4], Lthigh, Lshank)
                InitialDataL[i, 5:7] = get_hip_point(motion[i, 2], motion[i, 3], motion[i, 4], Lthigh, Lshank)
                InitialDataL[i, 7:9] = get_hip_point(motion[Lto[k+1], 2], motion[Lto[k+1], 3], motion[Lto[k+1], 4], Lthigh, Lshank)
            
            
            time_sw = np.linspace(0, (Lhs[k]-Lto[k])*delta,  Lhs[k]-Lto[k])
            time_st = np.linspace(0, (Lto[k+1]-Lhs[k])*delta,  Lto[k+1]-Lhs[k])
            
            time_swn = np.linspace(0, (Lhs[k]-Lto[k])*delta,  100)
            time_stn = np.linspace(0, (Lto[k+1]-Lhs[k])*delta,  100)
            
            Lswingx[k, :] = np.interp(time_swn, time_sw, InitialDataL[Lto[k]:Lhs[k], 5])
            Lswingy[k, :] = np.interp(time_swn, time_sw, InitialDataL[Lto[k]:Lhs[k], 6])
            Lswingt[k, :] = time_swn
            
            Lstancex[k, :] = np.interp(time_stn, time_st, InitialDataL[Lhs[k]:Lto[k+1], 5])
            Lstancey[k, :] = np.interp(time_stn, time_st, InitialDataL[Lhs[k]:Lto[k+1], 6])
            Lstancet[k, :] = time_stn
            
    else:
        for k in range(Lgait-1):
            for i in range(Lhs[k], Lto[k]):
                InitialDataL[i, 0] = 1
                InitialDataL[i, 1] = (i-Lhs[k])*delta
                InitialDataL[i, 2] = (Lto[k]-Lhs[k])*delta
                InitialDataL[i, 3:5] = get_hip_point(motion[Lhs[k], 2], motion[Lhs[k], 3], motion[Lhs[k], 4], Lthigh, Lshank)
                InitialDataL[i, 5:7] = get_hip_point(motion[i, 2], motion[i, 3], motion[i, 4], Lthigh, Lshank)
                InitialDataL[i, 7:9] = get_hip_point(motion[Lto[k], 2], motion[Lto[k], 3], motion[Lto[k], 4], Lthigh, Lshank)
            for j in range(Lto[k], Lhs[k+1]):
                InitialDataL[j, 0] = 0
                InitialDataL[j, 1] = (j-Lto[k])*delta
                InitialDataL[j, 2] = (Lhs[k+1]-Lto[k])*delta
                InitialDataL[j, 3:5] = get_ankle_point(motion[Lto[k], 2], motion[Lto[k], 3], motion[Lto[k], 4], Lthigh, Lshank)
                InitialDataL[j, 5:7] = get_ankle_point(motion[j, 2], motion[j, 3], motion[j, 4], Lthigh, Lshank)
                InitialDataL[j, 7:9] = get_ankle_point(motion[Lhs[k+1], 2], motion[Lhs[k+1], 3], motion[Lhs[k+1], 4], Lthigh, Lshank)
                
            time_sw = np.linspace(0, (Lhs[k+1]-Lto[k])*delta,  Lhs[k+1]-Lto[k])
            time_st = np.linspace(0, (Lto[k]-Lhs[k])*delta,  Lto[k]-Lhs[k])
            
            time_swn = np.linspace(0, (Lhs[k+1]-Lto[k])*delta,  100)
            time_stn = np.linspace(0, (Lto[k]-Lhs[k])*delta,  100)
            
            Lswingx[k, :] = np.interp(time_swn, time_sw, InitialDataL[Lto[k]:Lhs[k+1], 5])
            Lswingy[k, :] = np.interp(time_swn, time_sw, InitialDataL[Lto[k]:Lhs[k+1], 6])
            Lswingt[k, :] = time_swn
            
            Lstancex[k, :] = np.interp(time_stn, time_st, InitialDataL[Lhs[k]:Lto[k], 5])
            Lstancey[k, :] = np.interp(time_stn, time_st, InitialDataL[Lhs[k]:Lto[k], 6])
            Lstancet[k, :] = time_stn
            
    if Rto[0] < Rhs[0]:
        for p in range(Rgait-1):
            for j in range(Rto[p], Rhs[p]):
                InitialDataR[j, 0] = 0
                InitialDataR[j, 1] = (j-Rto[p])*delta
                InitialDataR[j, 2] = (Rhs[p]-Rto[p])*delta
                InitialDataR[j, 3:5] = get_ankle_point(motion[Rto[p], 2], motion[Rto[p], 6], motion[Rto[p], 7], Lthigh, Lshank)
                InitialDataR[j, 5:7] = get_ankle_point(motion[j, 2], motion[j, 6], motion[j, 7], Lthigh, Lshank)
                InitialDataR[j, 7:9] = get_ankle_point(motion[Rhs[p], 2], motion[Rhs[p], 6], motion[Rhs[p], 7], Lthigh, Lshank)
            for i in range(Rhs[p], Rto[p+1]):
                InitialDataR[i, 0] = 1
                InitialDataR[i, 1] = (i-Rhs[p])*delta
                InitialDataR[i, 2] = (Rto[p+1]-Rhs[p])*delta
                InitialDataR[i, 3:5] = get_hip_point(motion[Rhs[p], 2], motion[Rhs[p], 6], motion[Rhs[p], 7], Lthigh, Lshank)
                InitialDataR[i, 5:7] = get_hip_point(motion[i, 2], motion[i, 6], motion[i, 7], Lthigh, Lshank)
                InitialDataR[i, 7:9] = get_hip_point(motion[Rto[p+1], 2], motion[Rto[p+1], 6], motion[Rto[p+1], 7], Lthigh, Lshank)
                
            time_sw = np.linspace(0, (Rhs[p]-Rto[p])*delta,  Rhs[p]-Rto[p])
            time_st = np.linspace(0, (Rto[p+1]-Rhs[p])*delta,  Rto[p+1]-Rhs[p])
            
            time_swn = np.linspace(0, (Rhs[p]-Rto[p])*delta,  100)
            time_stn = np.linspace(0, (Rto[p+1]-Rhs[p])*delta,  100)
            
            Rswingx[p, :] = np.interp(time_swn, time_sw, InitialDataR[Rto[p]:Rhs[p], 5])
            Rswingy[p, :] = np.interp(time_swn, time_sw, InitialDataR[Rto[p]:Rhs[p], 6])
            Rswingt[p, :] = time_swn
            
            Rstancex[p, :] = np.interp(time_stn, time_st, InitialDataR[Rhs[p]:Rto[p+1], 5])
            Rstancey[p, :] = np.interp(time_stn, time_st, InitialDataR[Rhs[p]:Rto[p+1], 6])
            Rstancet[p, :] = time_stn
                
    else:
        for p in range(Rgait-1):
            for i in range(Rhs[p], Rto[p]):
                InitialDataR[i, 0] = 1
                InitialDataR[i, 1] = (i-Rhs[p])*delta
                InitialDataR[i, 2] = (Rto[p]-Rhs[p])*delta
                InitialDataR[i, 3:5] = get_hip_point(motion[Rhs[p], 2], motion[Rhs[p], 6], motion[Rhs[p], 7], Lthigh, Lshank)
                InitialDataR[i, 5:7] = get_hip_point(motion[i, 2], motion[i, 6], motion[i, 7], Lthigh, Lshank)
                InitialDataR[i, 7:9] = get_hip_point(motion[Rto[p], 2], motion[Rto[p], 6], motion[Rto[p], 7], Lthigh, Lshank)
            for j in range(Rto[p], Rhs[p+1]):
                InitialDataR[j, 0] = 0
                InitialDataR[j, 1] = (j-Rto[p])*delta
                InitialDataR[j, 2] = (Rhs[p+1]-Rto[p])*delta
                InitialDataR[j, 3:5] = get_ankle_point(motion[Rto[p], 2], motion[Rto[p], 6], motion[Rto[p], 7], Lthigh, Lshank)
                InitialDataR[j, 5:7] = get_ankle_point(motion[j, 2], motion[j, 6], motion[j, 7], Lthigh, Lshank)
                InitialDataR[j, 7:9] = get_ankle_point(motion[Rhs[p+1], 2], motion[Rhs[p+1], 6], motion[Rhs[p+1], 7], Lthigh, Lshank)
                
            time_sw = np.linspace(0, (Rhs[p+1]-Rto[p])*delta,  Rhs[p+1]-Rto[p])
            time_st = np.linspace(0, (Rto[p]-Rhs[p])*delta,  Rto[p]-Rhs[p])
            
            time_swn = np.linspace(0, (Rhs[p+1]-Rto[p])*delta,  100)
            time_stn = np.linspace(0, (Rto[p]-Rhs[p])*delta,  100)
            
            Rswingx[p, :] = np.interp(time_swn, time_sw, InitialDataR[Rto[p]:Rhs[p+1], 5])
            Rswingy[p, :] = np.interp(time_swn, time_sw, InitialDataR[Rto[p]:Rhs[p+1], 6])
            Rswingt[p, :] = time_swn
            
            Rstancex[p, :] = np.interp(time_stn, time_st, InitialDataR[Rhs[p]:Rto[p], 5])
            Rstancey[p, :] = np.interp(time_stn, time_st, InitialDataR[Rhs[p]:Rto[p], 6])
            Rstancet[p, :] = time_stn
    
    if write_sign == True:
        
        with open(store_path+'InitialDataL_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(0, num_nodes-2*ext_nodes):
                for g in range(9):
                    StringP += str(InitialDataL[r + ext_nodes, g])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
        
        with open(store_path+'InitialDataR_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(0, num_nodes-2*ext_nodes):
                for g in range(9):
                    StringP += str(InitialDataR[r + ext_nodes, g])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
            
        Lhs_s = Lhs[(Lhs >= ext_nodes) & (Lhs <= num_nodes-ext_nodes)]
        Lto_s = Lto[(Lto >= ext_nodes) & (Lto <= num_nodes-ext_nodes)]
        
        if Lhs_s[0] < Lto_s[0]:
            with open(store_path+'Lhs_Lto_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
                StringP = ''
                for r in range(np.min([len(Lhs_s), len(Lto_s)])):
                    StringP += str(Lhs_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += str(Lto_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += '\n'
                outfile.write(StringP)
        else:
            with open(store_path+'Lto_Lhs_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
                StringP = ''
                for r in range(np.min([len(Lhs_s), len(Lto_s)])):
                    StringP += str(Lto_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += str(Lhs_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += '\n'
                outfile.write(StringP)
                
        Rhs_s = Rhs[(Rhs >= ext_nodes) & (Rhs <= num_nodes-ext_nodes)]
        Rto_s = Rto[(Rto >= ext_nodes) & (Rto <= num_nodes-ext_nodes)]
        
        if Rhs_s[0] < Rto_s[0]:
            with open(store_path+'Rhs_Rto_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
                StringP = ''
                for r in range(np.min([len(Rhs_s), len(Rto_s)])):
                    StringP += str(Rhs_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += str(Rto_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += '\n'
                outfile.write(StringP)
        else:
            with open(store_path+'Rto_Rhs_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
                StringP = ''
                for r in range(np.min([len(Rhs_s), len(Rto_s)])):
                    StringP += str(Rto_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += str(Rhs_s[r]-ext_nodes)
                    StringP += ' '
                    StringP += '\n'
                outfile.write(StringP)
                
                
        with open(store_path+'Lswingx_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Lgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Lswingx[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)   
            
        with open(store_path+'Lswingy_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Lgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Lswingy[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP) 
            
        with open(store_path+'Lswingt_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Lgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Lswingt[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path+'Lstancex_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Lgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Lstancex[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path+'Lstancey_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Lgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Lstancey[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path+'Lstancet_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Lgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Lstancet[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
            
            
        with open(store_path+'Rswingx_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Rgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Rswingx[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)   
            
        with open(store_path+'Rswingy_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Rgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Rswingy[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP) 
            
        with open(store_path+'Rswingt_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Rgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Rswingt[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path+'Rstancex_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Rgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Rstancex[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path+'Rstancey_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Rgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Rstancey[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
            
        with open(store_path+'Rstancet_'+str(st_walking)
                                +'_'+str(ed_walking)+'_'+str(int(100/every_node))+'HZ.txt', 'w') as outfile:
            StringP = ''
            for r in range(Rgait-2*ext_gait):
                for q in range(100):
                    StringP += str(Rstancet[r+ext_gait, q])
                    StringP += ' '
                StringP += '\n'
            outfile.write(StringP)
        
    if plot_sign ==True:
        Gait_info = np.zeros(6)
        Gait_info_std = np.zeros(6)
        
        Gait_info[0] = np.mean(np.diff(Lhs))*delta
        Gait_info_std[0] = np.std(np.diff(Lhs))*delta
        
        Gait_info[1] = np.mean(Lto[2:-2] - Lhs[1:-3])*delta
        Gait_info_std[1] = np.std(Lto[2:-2] - Lhs[1:-3])*delta
        
        Gait_info[2] = np.mean(Lhs[2:-2] - Lto[2:-2])*delta
        Gait_info_std[2] = np.std(Lhs[2:-2] - Lto[2:-2])*delta
        
        Gait_info[3] = np.mean(np.diff(Rhs))*delta
        Gait_info_std[3] = np.std(np.diff(Rhs))*delta
        
        Gait_info[4] = np.mean(Rto[1:] - Rhs[:-1])*delta
        Gait_info_std[4] = np.std(Rto[1:] - Rhs[:-1])*delta
        
        Gait_info[5] = np.mean(Rhs - Rto)*delta
        Gait_info_std[5] = np.std(Rhs - Rto)*delta
        
        num_par = 3
        index = np.arange(num_par)
        fig2 = plt.figure(figsize=(8, 6))
        ax = fig2.add_subplot(1, 1, 1)
        width = 0.3
        
        p1 = ax.bar(index, Gait_info[:3], width, color='r', bottom=0, yerr=Gait_info_std[:3])
        p2 = ax.bar(index+width, Gait_info[3:6], width, color='y', bottom=0, yerr=Gait_info_std[3:6])
        
        ax.set_title('Gait Period Info (s)', fontsize=14)
        ax.set_xticks(index + width / 2)
        ax.set_xticklabels(('Full Gait', 'Stance', 'Swing'), fontsize=14)
        ax.legend((p1[0], p2[0]), ('Left', 'Right'), fontsize=14)
        plt.show()
        
        Lstance_ind = np.where(InitialDataL[:, 0] == 1)[0]
        Lswing_ind = np.where(InitialDataL[:, 0] == 0)[0]
        
        Rstance_ind = np.where(InitialDataR[:, 0] == 1)[0]
        Rswing_ind = np.where(InitialDataR[:, 0] == 0)[0]
        
        fig4 = plt.figure(figsize=(14, 8))
        
        ax1 = fig4.add_subplot(2, 2, 1)
        plt.ylabel('Pelvis x (m)', fontsize = 14)
        ax2 = fig4.add_subplot(2, 2, 2)
        plt.ylabel('Pelvis y (m)', fontsize = 14)
        ax3 = fig4.add_subplot(2, 2, 3)
        plt.ylabel('Ankle x (m)', fontsize = 14)
        plt.xlabel('Gait period (s)', fontsize = 14)
        ax4 = fig4.add_subplot(2, 2, 4)
        plt.ylabel('Ankle y (m)', fontsize = 14)
        plt.xlabel('Gait period (s)', fontsize = 14)
        
        ax1.plot(InitialDataL[Lstance_ind, 1], InitialDataL[Lstance_ind, 5], '.', label='Left Leg')
        ax1.plot(InitialDataR[Rstance_ind, 1], InitialDataR[Rstance_ind, 5], '.', label='Right Leg')
        
        ax2.plot(InitialDataL[Lstance_ind, 1], InitialDataL[Lstance_ind, 6], '.', label='Left Leg')
        ax2.plot(InitialDataR[Rstance_ind, 1], InitialDataR[Rstance_ind, 6], '.', label='Right Leg')
        
        ax3.plot(InitialDataL[Lswing_ind, 1], InitialDataL[Lswing_ind, 5], '.', label='Left Leg')
        ax3.plot(InitialDataR[Rswing_ind, 1], InitialDataR[Rswing_ind, 5], '.', label='Right Leg')
        
        ax4.plot(InitialDataL[Lswing_ind, 1], InitialDataL[Lswing_ind, 6], '.', label='Left Leg')
        ax4.plot(InitialDataR[Rswing_ind, 1], InitialDataR[Rswing_ind, 6], '.', label='Right Leg')
        plt.legend(fontsize=14)
        plt.show()