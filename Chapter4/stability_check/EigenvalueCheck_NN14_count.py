#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 17:42:47 2018

Find Equillibium Point

@author: huawei
"""
import numpy as np
from numpy import sin, cos

def Afun(a):
    """
    Activation function
    """
    k = 50
    f = 0.7/k*np.log(1+np.exp(-k*a)) + a
    
    return f

def Cdfda(a):
    """
    Derivative of the function with respect to inputs
    """
    k = 50
    Len = a.size
    dfda = np.zeros((Len+1, Len))
    
    for i in range(Len):
        dfda[i, i] = 0.7/(1+np.exp(-k*a[i])) + 0.3
            
    return dfda


def Cdadw (f):
    """
    Derivative of the function with respect to weights
    """
    
    Len = f.size
    
    HN = hid_nodes
    
    dadw = np.zeros((HN, HN*Len))
    
    for i in range(HN):
        dadw[i, i*Len:(i+1)*Len] = f
             
    return dadw

def neurnet_con(x, xdel, p):
    """
    Controller Model, get torques and jacobian
    
    Torques = K*(X_ref - X)
    
    """
    
    PN = num_par
    S = num_states
    CO = conout
    D = delay_nodes
    cons = 1
    HL = hid_layer
    HN = hid_nodes
    
    input_size = len(x)+len(xdel)
    
    Input = np.hstack((x, xdel, np.array([cons])))

    av = np.zeros((HN, HL))
    fv = np.zeros((HN+1, HL))
    
    dtordp = np.zeros((CO, PN))
    dtordinp = np.zeros((CO, input_size))
    dtorda = np.zeros((HL, CO, HN))
    
    
    weight_b = p[0:(input_size+1)*HN].reshape((HN, input_size+1))
    
    if HL > 1:
        weight_hl = p[(input_size+1)*HN:-CO*(HN+1)].reshape((HL-1, HN, HN+1))
                
    weight_e = p[-CO*(HN+1):].reshape((CO, HN+1))
    
    av[:,0] = np.dot(weight_b, Input)
    
    for k in range(HL-1):
        
        fv[:-1,k] = Afun(av[:, k])
        fv[-1,k] = cons
        av[:,k+1] = np.dot(weight_hl[k,:,:], fv[:,k])
              
    fv[:-1,HL-1] = Afun(av[:, HL-1])       
    fv[-1,HL-1] = cons
      
    Tor = np.dot(weight_e, fv[:,HL-1])
    
    dtordp[:, -CO*(HN+1):] = np.array([np.hstack([fv[:, HL-1], np.zeros((HN+1))]),
                                       np.hstack([np.zeros((HN+1)), fv[:, HL-1]])])
                          
    for l in range(HL):
        if l == 0:
            dtorda[HL-1-l,:,:] = np.dot(weight_e, Cdfda(av[:,HL-1-l]))            
        else:
            dtorda[HL-1-l,:,:] = np.dot(np.dot(dtorda[HL-l,:,:], weight_hl[HL-1-l,:,:]), Cdfda(av[:,HL-1-l]))
            
        if l < HL-1: 
            dtordp[:, -(HN*(HN+1)*(l+1)+CO*(HN+1)):-(HN*(HN+1)*l+CO*(HN+1))] = np.dot(dtorda[HL-1-l,:,:], Cdadw(fv[:,HL-2-l]))
        else:
            dtordp[:, :(S*(D+1)+1)*HN] = np.dot(dtorda[0, :, :], Cdadw(Input))
            
    dtordinp = np.dot(dtorda[0, :, :], weight_b)
    
    dtordx = dtordinp[:, :len(x)]
    dtordxdel = dtordinp[:, len(x):len(x)+len(xdel)]

    return Tor, dtordp, dtordx, dtordxdel


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
    
    Tor, dTordp, dTordx, dTordxdel = neurnet_con(x, xdel, p)

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

num_states = 4  # number of system states
conout = 2   # number of controller outputs
delay_nodes = 0  # zero delay nodes
cons = 1   # constant value of the neural network
hid_layer = 1  # number of hideen layer
hid_nodes = 4  # number of hidden nodes

# number of controller parameters
num_par = ((num_states*(delay_nodes+1)+1)*hid_nodes + (hid_layer-1)*(hid_nodes+1)*hid_nodes 
          + (hid_nodes+1)*conout)

eps = 10   # number of episodes  
Conindex = 0   # check the best fit controller

# load experimental data and controller
Accel = np.loadtxt('../experimental_data/Subj05/SwayL000' + str(Trial+1) + '.txt')
X_exp = np.loadtxt('../experimental_data/Subj05/JointsGFL000' + str(Trial+1) + '.txt')
parameters = np.loadtxt('../experimental_data/Para_Winter_Subj05.txt')

folder = 'NN14/Analysis/Eps'+str(eps)+'/'      
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


xdel = np.array([])   # empty delay states
p = con   # resign controller
a = 0   # zero acceleration while calculating the eigenvlaues
noise = np.zeros(2)   # zero noise while calculating the eigenvalues

# initialize of the eigenvalue matrix
EigPrecentage = np.zeros((num_batch, num_batch))
RealEig = np.zeros((num_batch, num_batch))

# run over all the batch situations to calculate the eigenvalue
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
                
        index_sta = np.where(RealEig<=0)    # get the index of all negative eigenvalues 
        
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