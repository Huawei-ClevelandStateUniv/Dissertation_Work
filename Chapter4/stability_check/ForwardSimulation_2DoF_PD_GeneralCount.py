#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 00:12:54 2017

@author: huawei
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 15 11:25:37 2017

@author: huawei
"""

# Geneerate Ode Function
import numpy as np
from numpy import cos, sin, sqrt, mean, square
from scipy.interpolate import interp1d

#from numpy import square, mean, sqrt
     
def ITP(Y, con, time):
    """
    Dynamics of the close-loop system with feedback controller
    
    """
    
    l_L = parameter[0]
    I_T = parameter[6]/scaling
    g = 9.81
    d_T = parameter[2]
    d_L = parameter[1]
    I_L = parameter[5]/scaling
    m_L = parameter[3]/scaling
    m_T = parameter[4]/scaling
    
    kp00 = con[0]
    kp01 = con[1]
    
    kp10 = con[2]
    kp11 = con[3]

              
    ref_a = con[4]
    ref_h = con[5]

               
    theta_a = Y[0]
    theta_h = Y[1]
    omega_a = Y[2]
    omega_h = Y[3]
    
    if time >= duration:
        a = func_acc(duration)
    else:
        a = func_acc(time)
    
    M = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, (I_L + I_T + d_L**2*m_L + m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)), (I_T + d_T*m_T*(d_T + l_L*cos(theta_h)))],
      [0, 0, (I_T + d_T*m_T*(d_T + l_L*cos(theta_h))), (I_T + d_T**2*m_T)]])
    
    ForceVec = np.matrix([[omega_a], [omega_h], [(a*d_L*m_L*cos(theta_a) + a*d_T*m_T*cos(theta_a + theta_h) 
            + a*l_L*m_T*cos(theta_a) + d_L*g*m_L*sin(theta_a) + d_T*g*m_T*sin(theta_a + theta_h) 
            - d_T*l_L*m_T*omega_a**2*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*sin(theta_h) 
            + g*l_L*m_T*sin(theta_a) - kp00*(theta_a-ref_a) - kp01*omega_a)],
    
                [(a*d_T*m_T*cos(theta_a + theta_h) + d_T*g*m_T*sin(theta_a + theta_h) 
            - d_T*l_L*m_T*omega_a**2*sin(theta_h) 
            - (kp10*(theta_h-ref_h) + kp11*omega_h))]])
    
    
    M_rev = M.I
    
    DY =np.dot(M_rev, ForceVec)
    DY = np.asarray(DY).reshape(-1)
    
    return DY


Subj = 5   # subject number
Trial = 2  # experiment trial number 

Eps = 4   # number of episodes 

num_nodes = 2001  # nodes of forward simulation 
duration = 40.0  # duration of forward simulation

start_nodes = 13000  # starting nodes for the forward simulation

scaling = 1.0    # scaling of the dynamic model
num_states = 4   # number of system states

StoreInd = 'True'  # change to 'True' if you want to store analysis result

# load experimental data and controller
Accel_String = ('../experimental_data/Subj05/SwayL000' + str(Trial+1) + '.txt')
X_String = ('../experimental_data/Subj05/JointsGFL000' + str(Trial+1) + '.txt')
            
accel_meas_full = np.loadtxt(Accel_String)
traj_full = np.loadtxt(X_String)

traj = traj_full[start_nodes:start_nodes+num_nodes, 1:5]*3.1416/180.0
time = np.linspace(0,duration,num_nodes)

# initialize the forward simulation trajectories
ode_nodes = (num_nodes-1)*4+1
forward_traj = np.zeros((ode_nodes, 4))
interval_ode = duration/(ode_nodes-1)
time_ode = np.linspace(0, duration, ode_nodes)

Para_string = ('../experimental_data/Para_Winter_Subj05.txt')
parameter = np.loadtxt(Para_string)
func_acc = interp1d(time, -accel_meas_full[start_nodes:start_nodes+num_nodes, 3])

num_ind = 1
num_eps = 4     # total number of episodes trials

# calcluate the minimum and maximum values of the experimental data 
Max = np.max(traj_full[8000:15000, 1:5], axis = 0)*3.1416/180
Min = np.min(traj_full[8000:15000, 1:5], axis = 0)*3.1416/180

num_batch = 11    # number of select nodes inside the experimental data

# get the batch nodes
Bt1 = np.linspace(Min[0], Max[0], num_batch, endpoint=True)
Bt2 = np.linspace(Min[1], Max[1], num_batch, endpoint=True)
Bt3 = np.linspace(Min[2], Max[2], num_batch, endpoint=True)
Bt4 = np.linspace(Min[3], Max[3], num_batch, endpoint=True)

# initialize the RMS matrix
RMS = np.zeros((num_batch, num_batch, num_batch, num_batch))

# run over all episode situations
for eps in range(num_eps):
    
    Conindex = 0   # check the best fit controller

    if eps == 0:
        eps_r = 0
    elif eps == 1:
        eps_r = 2
    elif eps == 2:
        eps_r = 3
    elif eps == 3:
        eps_r = 4
        
    folder = 'PD/Analysis/Eps'+str(eps_r)
    
    ConFull = np.loadtxt(folder + '/Sort_Con.txt')
    
    con = ConFull[Conindex,:]
    
    # run over all the batch situations to calculate the eigenvalue
    for t1 in range(num_batch):
        for t2 in range(num_batch):
            for t3 in range(num_batch):
                for t4 in range(num_batch):
                    
                    init = np.array([Bt1[t1], Bt2[t2], Bt3[t3], Bt4[t4]])
                    forward_traj[0, :] = init
                    
                    for k in range(ode_nodes-1):
                        DY = ITP(forward_traj[k, :], con, time_ode[k])
                        forward_traj[k+1, :] = forward_traj[k, :] + DY*interval_ode
                        
                        if (np.isnan(forward_traj[k+1, 0]) or np.isnan(forward_traj[k+1, 1])
                                 or np.isnan(forward_traj[k+1, 2]) or np.isnan(forward_traj[k+1, 3])):
                            RMS[t1, t2, t3, t4] = np.nan
                            break
                        else:
                            Vec_a = np.interp(time, time_ode, forward_traj[:, 0]) -traj[:, 0]
                            Vec_h = np.interp(time, time_ode, forward_traj[:, 1]) -traj[:, 1]
                            Vec_RMS = np.hstack((Vec_a, Vec_h))
                            RMS[t1, t2, t3, t4] = sqrt(mean(mean(square(Vec_RMS))))
    
    # Write the array to disk
    with open(folder + 'RMSAll.txt', 'w') as outfile:
        # I'm writing a header here just for the sake of readability
        # Any line starting with "#" will be ignored by numpy.loadtxt
        outfile.write('# Array shape: {0}\n'.format(RMS.shape))
    
        # Iterating through a ndimensional array produces slices along
        # the last axis. This is equivalent to data[i,:,:] in this case
        for r1 in range(num_batch):
            for r2 in range(num_batch):
                
                # Writing out a break to indicate different slices...
                outfile.write('# Ankle and Hip Angle: {0}\n'.format([Bt1[r1], Bt2[r2]]))
                              
                StringP = ''
                              
                for r3 in range(num_batch):
                    for r4 in range(num_batch):
                        StringP += str(RMS[r1, r2, r3, r4])
                        StringP += ' '
                    StringP += '\n'
                StringP += '\n'
                outfile.write(StringP)
    