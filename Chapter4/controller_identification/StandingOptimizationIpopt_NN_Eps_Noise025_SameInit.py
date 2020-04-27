#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  1 15:58:34 2017

@author: huawei
"""

import ipopt
import numpy as np
from StandingModel_NN_LeakyReLU_Eps_Noise_SameInit import Model
from numpy import sqrt, mean, square
import time
import os
import multiprocessing

num_trials = 10 # total number of optimizations 

# prepare for the parallel computing
data = []
for q in range(num_trials):
    data.append([q])
data = tuple(data)

subj = 5  # subject number of the identification
num_nodes = 5001   # total number of collocation nodes
duration = 100.0  # total duration of identification experimental data
interval = duration / (num_nodes - 1)
start_nodes = 8000  # the starting node of the identification period in experimental data

num_states = 4  # number of states of the standing balance system
num_cons = 4  # number of constraints in each collocation node
Scaling = 100.0  # scaling number for the dynamic contraints

num_repeat = 5    # number of episodes in the sotchastic optimization

delay_nodes = 0  # number of delay nodes as inputs of the neural network
conout = 2   # number of controller output
hid_layer = 1  # number of hidden layers
hid_nodes = 2  # number of hidden nodes in each hidden layers

NoiVal = 0.25  # amplitude of the random noise

# number of parameters of the identification controller 
num_par = ((num_states*(delay_nodes+1)+1)*hid_nodes + (hid_layer-1)*(hid_nodes+1)*hid_nodes 
          + (hid_nodes+1)*conout)

# index of the the ankle and hip joint trajectories in the experiemntal data
itheta_a = np.linspace(0, num_states*num_nodes*num_repeat, num_nodes*num_repeat,
                       endpoint=False, dtype=int)
itheta_h = np.linspace(0, num_states*num_nodes*num_repeat, num_nodes*num_repeat,
                       endpoint=False, dtype=int) + 1
                       
# calculate the bounds of the identifying parameters    
lb_x = np.zeros((num_states*num_nodes*num_repeat)) - 2.0
lb_x[itheta_a] = np.zeros((num_nodes*num_repeat)) - 1.0
lb_x[itheta_h] = np.zeros((num_nodes*num_repeat)) - 1.0
              
ub_x = np.zeros((num_states*num_nodes*num_repeat)) + 2.0
ub_x[itheta_a] = np.zeros((num_nodes*num_repeat)) + 1.0
ub_x[itheta_h] = np.zeros((num_nodes*num_repeat)) + 1.0

cl = np.zeros((num_cons*(num_nodes -1)*num_repeat + (num_repeat-1)*num_states))
cu = np.zeros((num_cons*(num_nodes -1)*num_repeat + (num_repeat-1)*num_states))

# define the path for saving data
Dictionary1 = 'Results_150_250/Subj0'+str(subj)+'/Pert1_GFL_NN'+str(hid_layer)+str(hid_nodes)+'_Eps'+str(num_repeat)+'_Noise025_SameInit/'
if not os.path.exists(Dictionary1):
    os.makedirs(Dictionary1)
    
Accel_String1 = ('../experimental_data/Subj0'+str(subj)+'/SwayL0003.txt')
X_String1 = ('../experimental_data/Subj0'+str(subj)+'/JointsGFL0003.txt')

Para_string = ('../experimental_data/Model Parameter_2DoF/Para_Winter_Subj0'+str(subj)+'.txt')

accel_meas_full1 = np.loadtxt(Accel_String1, usecols=(3))
x_meas_full1 = np.loadtxt(X_String1, usecols=(1, 2, 3, 4,))
parameter = np.loadtxt(Para_string)

Con_init1 = np.loadtxt(('../experimental_data/'
                        +'NNTranningGains/Weights_Subj0'+str(subj)
                        +'T03_NN'+str(hid_layer)+str(hid_nodes)+'.txt'))

accel_meas1 = -accel_meas_full1[start_nodes:start_nodes+num_nodes]
x_meas1 = x_meas_full1[start_nodes:start_nodes+num_nodes, :]*3.1416/180.0

x_meas_vec1 = x_meas1.T.flatten('F')

x_exp1 = np.array([])
acc_exp1 = np.array([])

lb_c1 = Con_init1 - 5
ub_c1 = Con_init1 + 5

lb1 = np.hstack((lb_x, lb_c1))
ub1 = np.hstack((ub_x, ub_c1))

for r in range(num_repeat):
    
    x_exp1 = np.hstack((x_exp1, x_meas_vec1))
    acc_exp1 = np.hstack((acc_exp1, accel_meas1))

# define a function to perform the optimization
def do_optimization(j):
        
    
    np.random.seed()
    noise1 = NoiVal/Scaling - 2.0*NoiVal/Scaling*np.random.random((num_nodes*num_repeat, int(num_states/2)))
    
    # initialize the optimization problem
    nlp1 = ipopt.problem(
                n=num_states*num_nodes*num_repeat + num_par,
                m=num_cons*(num_nodes -1)*num_repeat + (num_repeat-1)*num_states,
                problem_obj=Model(x_exp1, acc_exp1, num_repeat, num_nodes, num_states,
                                   delay_nodes, interval, hid_layer, hid_nodes,
                                   parameter, noise1, scaling =Scaling,
                                   integration_method='midpoint'),
                lb=lb1,
                ub=ub1,
                cl=cl,
                cu=cu
                )
    
    nlp1.addOption(b'linear_solver', b'MUMPS')  # use 'MA86' for faster speed, but less likely to find solutions
    nlp1.addOption(b'max_iter', 3000)
    nlp1.addOption(b'tol', 1e-6)
    nlp1.addOption(b'acceptable_tol', 1e-4)
    nlp1.addOption(b'max_cpu_time', 6e+4)
    
    # randomize the initial guess of the control parameters
    np.random.seed()
    unknown_p1 = Con_init1 + (1 - 2*np.random.random(num_par))
    x0_1 =  np.hstack((x_exp1, unknown_p1))
    
    
    for s in range(num_states):
        np.random.seed()             # change seed every iteration
        ra = np.random.random(1)     # generate a random number
        
        # initialize every state parameters of initial conditions in the range
        # of [0.9, 1.1] time experimental data
        for r in range(num_repeat):  
            x0_1[r*num_states*num_nodes+s] = 0.9*x_meas1[0,s] + 0.2*ra*np.abs(x_meas1[0,s])
    
    # measure the time cost of the optimization
    start_time = time.time()
    x, info = nlp1.solve(x0_1)
    TimeCost = time.time() - start_time
    
    # calculate the RMS between optimized trajectory and experimental data
    Vec_a = x_exp1[itheta_a]-x[itheta_a]
    Vec_h = x_exp1[itheta_h]-x[itheta_h]
    Dictionary = Dictionary1
    
    Vec_RMS = np.hstack((Vec_a, Vec_h))
    RMS = sqrt(mean(mean(square(Vec_RMS))))
    Sta = info['status']
    ReCt = info['g']
    
    # save identified results into txt files
    FitName = Dictionary + 'TrajectoryResult_'+ str(j) + '.txt'     
    with open(FitName,'w') as Outfile:
        for m in range(0,num_nodes*num_repeat):
            StringP = ""
            for n in range(0,num_states):
                StringP += str(x[m*num_states + n])
                StringP += " "
            StringP += "\n"
            Outfile.write(StringP)
            
    RcstName = Dictionary + 'ResiduleConstraints_'+ str(j) + '.txt'
    with open(RcstName,'w') as Outfile:
        StringP = ""
        for m in range(0,len(cl)):
            StringP += str(ReCt[m])
            StringP += "\n"
        Outfile.write(StringP)
            

    ContrName = Dictionary + 'Controller_'+ str(j) + '.txt'
    with open(ContrName,'w') as Outfile:
        StringP = ""
        for q in range(0,num_par):
            StringP += str(x[num_nodes*num_states*num_repeat + q])
            StringP += " "
        StringP += "\n"
        Outfile.write(StringP)
    
    RMS_Sta_Name = Dictionary + 'RMS_Sta_'+ str(j) + '.txt'
    with open(RMS_Sta_Name,'w') as Outfile:
        StringP = ""
        StringP += str(RMS)
        StringP += "\n"
        StringP += str(Sta)
        StringP += "\n"
        StringP += str(TimeCost)
        StringP += "\n"
        Outfile.write(StringP)
    
    return Sta  


def mp_handler():
    p = multiprocessing.Pool(4)
    x_res = p.map_async(do_optimization, data)
    x_res.get()

if __name__ == '__main__':
    mp_handler()
