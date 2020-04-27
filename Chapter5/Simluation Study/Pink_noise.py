#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 24 22:27:40 2019

@author: huawei
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import mlab
from scipy import signal


def one_over_f(f, knee, alpha):
    desc = np.ones_like(f)
    desc[f<KNEE] = np.abs((f[f<KNEE]/KNEE)**(-alpha))
    desc[0] = 1
    return desc

for k in range(50):
    #white_noise_sigma =  1.5e-5 #mK * sqrt(s)  Noise power for ankle joint
    white_noise_sigma =  5e-6 #mK * sqrt(s) Noise power for hip joint
    
    SFREQ = 100 #Hz
    KNEE = 10 #Hz
    ALPHA = 1
    N = 10001 # 60 seconds
    
    #generate white noise in time domain
    wn=np.random.normal(0.,white_noise_sigma*np.sqrt(SFREQ),N)
    
    #shaping in freq domain
    s = np.fft.rfft(wn)
    f = np.fft.fftfreq(N, d=1./SFREQ)[:len(s)]
    f[-1]=np.abs(f[-1])
    fft_sim = s * one_over_f(f, KNEE, ALPHA)
    T_sim = np.fft.irfft(fft_sim)
    
    #PSD - 50 seconds window
    NFFT = int(SFREQ*50)
    s_sim, f_sim  = mlab.psd(T_sim, NFFT=NFFT, Fs=SFREQ, scale_by_freq=True)
    
    
    #plot
#    plt.figure()
#    plt.plot(f_sim, np.sqrt(s_sim), label='sim')
#    plt.loglog(f_sim, one_over_f(f_sim, KNEE, ALPHA) * white_noise_sigma*1e3*np.sqrt(2), 'r',label='noise model')
#    plt.vlines(KNEE,*plt.ylim())
#    plt.grid(); plt.xlabel('Freq'); plt.title('Amplitude spectrum'); plt.legend()
    
    b, a = signal.butter(1, 0.15/(100/2))
    
    T_sim_filt = signal.filtfilt(b, a, T_sim)
    
    
#    plt.figure()
#    plt.plot(T_sim)
#    plt.plot(T_sim_filt, 'r')
    
    noise_rms = np.sqrt(sum(T_sim**2)/N)*180/3.1416
    noise_filt_rms = np.sqrt(sum(T_sim_filt**2)/N)*180/3.1416
    
    pink_noise = 'ForwardSimulation_GoodWorth/pink_noise2_10000_'+str(k)+'.txt'     
    with open(pink_noise,'w') as Outfile:
        StringP = ""
        for m in range(0, N-1):
            StringP += str(T_sim_filt[m])
            StringP += "\n"
        Outfile.write(StringP)