#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 17:57:54 2019

@author: huawei
"""

import numpy as np

def polynomial_value(L0, L1, T, t, Coef):
    
    num_par = len(Coef)
    
    if num_par == 1:
        Bk = Coef[0]
        Fit = (L0 + (Bk*(L1- L0))*(t/T))
        
    elif num_par == 2:
        Bk = Coef[0]
        Ck = Coef[1]
        Fit = (L0 + (Bk*(L1- L0))*(t/T) + Ck*(L1- L0)*(t/T)**2)
        
    elif num_par == 3:
        Bk = Coef[0]
        Ck = Coef[1]
        Dk = Coef[2]
        Fit = (L0 + (Bk*(L1- L0))*(t/T) + Ck*(L1- L0)*(t/T)**2
               + (Dk*(L1-L0))*(t/T)**3)
        
    elif num_par == 4:
        Bk = Coef[0]
        Ck = Coef[1]
        Dk = Coef[2]
        Ek = Coef[3]
        Fit = (L0 + (Bk*(L1- L0))*(t/T) + Ck*(L1- L0)*(t/T)**2
               + (Dk*(L1-L0))*(t/T)**3 + (Ek*(L1-L0))*(t/T)**4)
    elif num_par == 5:
        Bk = Coef[0]
        Ck = Coef[1]
        Dk = Coef[2]
        Ek = Coef[3]
        Fk = Coef[4]
        Fit = (L0 + (Bk*(L1- L0))*(t/T) + Ck*(L1- L0)*(t/T)**2
               + (Dk*(L1-L0))*(t/T)**3 + (Ek*(L1-L0))*(t/T)**4
               + (Fk*(L1-L0))*(t/T)**5)
    elif num_par == 6:
        Bk = Coef[0]
        Ck = Coef[1]
        Dk = Coef[2]
        Ek = Coef[3]
        Fk = Coef[4]
        Gk = Coef[5]
        Fit = (L0 + (Bk*(L1- L0))*(t/T) + Ck*(L1- L0)*(t/T)**2
               + (Dk*(L1-L0))*(t/T)**3 + (Ek*(L1-L0))*(t/T)**4
               + (Fk*(L1-L0))*(t/T)**5 + (Gk*(L1-L0))*(t/T)**6)
        
    return Fit