#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 17:42:47 2018

Find Equillibium Point

@author: huawei
"""
import numpy as np
from numpy import sin, cos

def controller(x, xdel, p):
    """
    Controller Model, get torques and jacobian
    
    Torques = K*(X_ref - X)
    
    """
    
    Tor = np.zeros(2)
    dTordp = np.zeros((2, len(p)))
    dTordx = np.zeros((2, 4))
    dTordxdel = np.zeros((2, len(xdel)))
    
    Kpa = -p[0]
    Kda = -p[1]
    Kph = -p[2]
    Kdh = -p[3]
    
    ra = p[4]
    rh = p[5]
    
    Tor[0] = Kpa*(ra-x[0]) + Kda*(-x[2])
    Tor[1] = Kph*(rh-x[1]) + Kdh*(-x[3])
    
    dTordp[0, 0] = (ra-x[0])
    dTordp[0, 1] = (-x[2])
    dTordp[0, 4] = Kpa
    
    dTordp[1, 2] = (rh-x[1])
    dTordp[1, 3] = (-x[3])
    dTordp[1, 5] = Kph
    
    dTordx[0, 0] = -Kpa
    dTordx[0, 2] = -Kda 
    
    dTordx[1, 1] = -Kph
    dTordx[1, 3] = -Kdh
    
    return Tor, dTordp, dTordx, dTordxdel

def dynamic_fun(x, xdel, xdot, p, a, noise):
    """
    Dynamics of the human standing balance model
    
    """

    l_L = parameters[0]
    I_T = parameters[6]/scaling
    g = 9.81
    d_T = parameters[2]
    d_L = parameters[1]
    I_L = parameters[5]/scaling
    m_L = parameters[3]/scaling
    m_T = parameters[4]/scaling
                         
    theta_a = x[0]
    theta_h = x[1]
    omega_a = x[2]
    omega_h = x[3]
    
    theta_a_dot = xdot[0]
    theta_h_dot = xdot[1]
    omega_a_dot = xdot[2]
    omega_h_dot = xdot[3]
    
    S = num_states
    PN = num_par
    
    f = np.zeros((S))
    dfdx = np.zeros((S, S))
    dfdxdel = np.zeros((S, len(xdel)))
    dfdxdot = np.zeros((S, S))
    dfdp = np.zeros((S, PN))
    
    Tor, dTordp, dTordx, dTordxdel = controller(x, xdel, p)

    f[0] = omega_a - theta_a_dot

    dfdx[0,2] = 1
    dfdxdot[0,0] = -1
    
    f[1] = omega_h - theta_h_dot
    
    dfdx[1,3] = 1
    dfdxdot[1,1] = -1
    
    f[2] = (a*d_L*m_L*cos(theta_a) + a*d_T*m_T*cos(theta_a + theta_h) 
            + a*l_L*m_T*cos(theta_a) + d_L*g*m_L*sin(theta_a) + d_T*g*m_T*sin(theta_a + theta_h) 
            - d_T*l_L*m_T*omega_a**2*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*sin(theta_h) 
            + g*l_L*m_T*sin(theta_a) - omega_a_dot*(I_L + I_T + d_L**2*m_L + m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)) 
            - omega_h_dot*(I_T + d_T*m_T*(d_T + l_L*cos(theta_h))) - Tor[0] -noise[0])
  
    
    dfdx[2,0] = (-a*d_L*m_L*sin(theta_a) - a*d_T*m_T*sin(theta_a + theta_h)
                    - a*l_L*m_T*sin(theta_a) + d_L*g*m_L*cos(theta_a) 
                    + d_T*g*m_T*cos(theta_a + theta_h) + g*l_L*m_T*cos(theta_a) - dTordx[0, 0])
                                   
    dfdx[2,1] = (-a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*cos(theta_h) + 2*d_T*l_L*m_T*omega_a_dot*sin(theta_h) 
                + d_T*l_L*m_T*omega_h_dot*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*cos(theta_h) - dTordx[0, 1])
                                             
    dfdx[2,2] = -2*d_T*l_L*m_T*omega_a*sin(theta_h) + d_T*l_L*m_T*(2*omega_a + 2*omega_h)*sin(theta_h) - dTordx[0, 2]
    
    dfdx[2,3] = d_T*l_L*m_T*(2*omega_a + 2*omega_h)*sin(theta_h) - dTordx[0, 3]
    
    dfdxdot[2,2] = -I_L - I_T - d_L**2*m_L - m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)
    dfdxdot[2,3] = -I_T - d_T*m_T*(d_T + l_L*cos(theta_h))
    
    
    dfdp[2,:] = -dTordp[0,:]
        
    f[3] =  (a*d_T*m_T*cos(theta_a + theta_h) + d_T*g*m_T*sin(theta_a + theta_h) 
            - d_T*l_L*m_T*omega_a**2*sin(theta_h) - (I_T + d_T**2*m_T)*omega_h_dot 
            - (I_T + d_T*m_T*(d_T + l_L*cos(theta_h)))*omega_a_dot - Tor[1] - noise[1])
    
    
    dfdx[3,0] = -a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) - dTordx[1, 0]
    
    dfdx[3,1] = (-a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*cos(theta_h) + d_T*l_L*m_T*omega_a_dot*sin(theta_h) - dTordx[1, 1])
                                                                           
    dfdx[3,2] = -2*d_T*l_L*m_T*omega_a*sin(theta_h) - dTordx[1, 2]
    dfdx[3,3] = - dTordx[1, 3]
        
    dfdxdot[3,2] = -I_T - d_T*m_T*(d_T + l_L*cos(theta_h))
    dfdxdot[3,3] = -I_T - d_T**2*m_T
    
    dfdp[3,:] = -dTordp[1,:]
    
    dfdxdel[2:4,:] = - dTordxdel
    
    
    return f, dfdx, dfdxdot
    

Subj = 5   # subject number
Trial = 2  # experiment trial number 
   
scaling = 1.0  # scaling of the dynamic model

num_states = 4  # number of the states
conout = 2      # number of controller outputs
delay_nodes = 0  # delay nodes of the controller

num_par = 6  # number of the control parameters

eps = 4   # number of episodes 
Conindex = 0  # check the best fit controller

# load experimental data and controller
Accel = np.loadtxt('../experimental_data/Subj05/SwayL000' + str(Trial+1) + '.txt')
X_exp = np.loadtxt('../experimental_data/Subj05/JointsGFL000' + str(Trial+1) + '.txt')
parameters = np.loadtxt('../experimental_data/Para_Winter_Subj05.txt')

folder = 'PD/Analysis/Eps'+str(eps)+'/'      
ConFull = np.loadtxt(folder + 'Sort_Con.txt')
con = ConFull[Conindex,:]

# calcluate the minimum and maximum values of the experimental data 
Max = np.max(X_exp[8000:15000, 1:5], axis = 0)*3.1416/180
Min = np.min(X_exp[8000:15000, 1:5], axis = 0)*3.1416/180

num_batch = 11  # number of select nodes inside the experimental data

# get the batch nodes
Bt1 = np.linspace(Min[0], Max[0], num_batch, endpoint=True)
Bt2 = np.linspace(Min[1], Max[1], num_batch, endpoint=True)
Bt3 = np.linspace(Min[2], Max[2], num_batch, endpoint=True)
Bt4 = np.linspace(Min[3], Max[3], num_batch, endpoint=True)


xdel = np.array([])  # empty delay states
p = con  # resign controller
a = 0  # zero acceleration while calculating the eigenvlaues
noise = np.zeros(2)  # zero noise while calculating the eigenvalues

# initialize of the eigenvalue matrix
EigPrecentage = np.zeros((num_batch, num_batch))
RealEig = np.zeros((num_batch, num_batch))

# ran over all the batch situations to calculate the eigenvalue
for i in range(num_batch):
    for j in range(num_batch):
        for k in range(num_batch):
            for l in range(num_batch):
                
                x = np.array([Bt1[i], Bt2[j], Bt3[k], Bt4[l]])
                xdot = np.array([Bt3[k], Bt4[l], 0, 0])
                
                f, dfdx, dfdxdot = dynamic_fun(x, xdel, xdot, p, a, noise)
                Mat_A = -np.dot(dfdx, np.linalg.inv(dfdxdot))
                Eig, Vec = np.linalg.eig(Mat_A)
                
                RealEig[k, l] = max(Eig.real)
        
        index_sta = np.where(RealEig<=0)  # get the index of all negative eigenvalues 
        
        # calculate the percentage of the all stateble eigenvalue nodes
        EigPrecentage[i, j] = len(index_sta[0])/(num_batch*num_batch)*100


# save the percentage of the all eigenvalue nodes
with open(folder + 'EigPercentage.txt','w') as Outfile:
    StringP = ""
    for n in range(num_batch):
        for m in range(num_batch):
            StringP += str(EigPrecentage[n, m])
            StringP += " "
        StringP += "\n"
    Outfile.write(StringP)