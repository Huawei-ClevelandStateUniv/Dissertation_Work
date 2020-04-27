#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 10:46:43 2017

Revised on Wed May 02 12:13:00 2018

@author: huawei

This program defined gait2dpi model for direct collocation trajectory optimization.
"""

import sys
sys.path.append('../')

import numpy as np
from gait2dpi_dGRF_LinContact import evaluate_autolev_rhs as autolev_rhs
from simulate import map_values_to_autolev_symbols
from scipy.sparse import find

class gait2dpi_model(object):
    def __init__(self, x_meas, x_normal_ref, vs_meas, num_nodes, num_states, index_open,
                    index_control, num_normal, num_par, interval, weight_mt,
                    weight_ms, weight_ts, weight_tm, weight_tp, index_normal,
                    index_impedance, constants_dict, scaling =1):
                
        self.x_meas = x_meas
        self.x_normal_ref = x_normal_ref
        self.vs_meas = vs_meas
        self.num_nodes = num_nodes
        self.num_states = num_states
        self.num_cons = num_states
        self.index_open = index_open
        self.index_control = index_control
        self.num_normal = num_normal
        self.num_par = num_par
        self.num_conspernode = num_states
    
        self.interval = interval
        self.scaling = scaling
        
        self.weight_mt = weight_mt
        self.weight_ms = weight_ms
        self.weight_ts = weight_ts
        self.weight_tm = weight_tm
        self.weight_tp = weight_tp
        
        self.index_normal = index_normal
        self.index_impedance = index_impedance
        
        
        # calculate index of human model joints
        self.i_joints = np.zeros((int(self.num_states/2), self.num_nodes))
        
        for h in range(int(self.num_states/2)):
            self.i_joints[h, :] = np.linspace(0, self.num_states*self.num_nodes,
                                  self.num_nodes, endpoint=False, dtype=int) + h
                         
        self.i_joints = self.i_joints.astype(int)
        
        # calcaulte index of open and normal joint torques
        self.num_control = len(index_control)
        self.num_open = len(index_open)
        
        self.i_openT = np.zeros((self.num_open, num_nodes))
        self.i_normalT = np.zeros((self.num_control, num_normal))
        
        index_openT_ST = self.num_states*self.num_nodes
        
        for i in range(self.num_open):
            
            self.i_openT[i, :] = np.linspace(index_openT_ST, index_openT_ST + self.num_nodes*self.num_open,
                                    self.num_nodes, endpoint=False, dtype=int) + i
                        
        self.i_openT = self.i_openT.astype(int)
                        
        index_normalT_ST = self.num_states*self.num_nodes + self.num_nodes*self.num_open
        
        for j in range(self.num_control):
            
            self.i_normalT[j, :] = np.linspace(index_normalT_ST,
                                              index_normalT_ST + self.num_normal*self.num_control,
                                              self.num_normal, endpoint=False, dtype=int) + j
        
        self.i_normalT = self.i_normalT.astype(int)
        
        # calculate index of impedance controlled joint motions at one direct collocation node
        self.index_control_motion = (self.index_control + 3).astype(int)
        self.index_open_motion = (self.index_open + 3).astype(int)
        
        self.constants_dict = map_values_to_autolev_symbols(constants_dict)

        
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        
        # tracking of joint motion
        f_motion = 0
        for h in range(int(self.num_states/2)):
            if h == 1:
                continue
            else:
                f_motion += np.sum((x[self.i_joints[h, :]] - self.x_meas[self.i_joints[h, :]])**2)
        
        # smoothing of joint motion
        fs_motion = 0
        for k in range(3, int(self.num_states/2)):
            fs_motion += np.sum(((x[self.i_joints[k, 1:]] - x[self.i_joints[k, :-1]])/self.interval)**2)

        # smoothing of joint open torques
        f_openT = 0
        for i in range(self.num_open):
            
            f_openT  += np.sum(((x[self.i_openT[i, 1:]] - x[self.i_openT[i, :-1]])/self.interval)**2)
            
        # smoothing of joint normal torques
        f_normalT = 0
        for j in range(self.num_control):
            
            f_normalT += np.sum(((x[self.i_normalT[j, 1:]] - x[self.i_normalT[j, :-1]])/self.interval)**2)
            
        # require semi-peroid of the normal joint torques
        f_peroid = np.sum((x[self.i_normalT[:, 0]] - x[self.i_normalT[:, -1]])**2)
        
        # minimize impedance control
        f_con = 0
        for i in range(self.num_nodes):
            
            tor, _, _ = self.impedance_control(x[i*self.num_states:(i+1)*self.num_states],
                                               self.x_meas[i*self.num_states:(i+1)*self.num_states],
                                                x[-self.num_par:], self.index_impedance[i, :])

            f_con += np.sum(tor**2)
        
        obj = (self.weight_mt*f_motion + self.weight_ms*fs_motion + self.weight_ts*(f_openT + f_normalT) 
               + self.weight_tm*f_con)/self.num_nodes + self.weight_tp*f_peroid
        
        return obj
    
	 
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        
        grad = np.zeros_like(x)
        
        # tracking of joint motion
        for h in range(int(self.num_states/2)):
            if h == 1:
                continue
            else:
                grad[self.i_joints[h, :]] = 2.0/self.num_nodes*self.weight_mt*(x[self.i_joints[h, :]] - self.x_meas[self.i_joints[h, :]])
                
        # smoothing joint motion
        for k in range(3, int(self.num_states/2)):
            grad[self.i_joints[k, 1:]] += 2.0/self.interval**2*(x[self.i_joints[k, 1:]] - x[self.i_joints[k, :-1]])/self.num_nodes*self.weight_ms
            grad[self.i_joints[k, :-1]] += -2.0/self.interval**2*(x[self.i_joints[k, 1:]] - x[self.i_joints[k, :-1]])/self.num_nodes*self.weight_ms
        
        # smoothing of open joint torques
        for i in range(self.num_open):
            
            grad[self.i_openT[i, 1:]] = 2.0/self.interval**2*(x[self.i_openT[i, 1:]] - x[self.i_openT[i, :-1]])/self.num_nodes*self.weight_ts
            grad[self.i_openT[i, :-1]] += -2.0/self.interval**2*(x[self.i_openT[i, 1:]] - x[self.i_openT[i, :-1]])/self.num_nodes*self.weight_ts
            
        # smoothing of normal joint torques
        for j in range(self.num_control):
            
            grad[self.i_normalT[j, 1:]] = 2.0/self.interval**2*(x[self.i_normalT[j, 1:]] - x[self.i_normalT[j, :-1]])/self.num_nodes*self.weight_ts
            grad[self.i_normalT[j, :-1]] += -2.0/self.interval**2*(x[self.i_normalT[j, 1:]] - x[self.i_normalT[j, :-1]])/self.num_nodes*self.weight_ts
            
        # semi-peroid of normal joint torques
        
        grad[self.i_normalT[:, 0]] += 2*(x[self.i_normalT[:, 0]] - x[self.i_normalT[:, -1]])*self.weight_tp
        grad[self.i_normalT[:, -1]] += -2*(x[self.i_normalT[:, 0]] - x[self.i_normalT[:, -1]])*self.weight_tp
        
        # Gradient of minimizing control output
        for i in range(self.num_nodes):
            
            tor, dt_dx, dt_dp = self.impedance_control(x[i*self.num_states:(i+1)*self.num_states],
                                            self.x_meas[i*self.num_states:(i+1)*self.num_states],
                                            x[-self.num_par:], self.index_impedance[i, :])
                    
            grad[i*self.num_states:(i+1)*self.num_states] += 2*np.dot(tor, dt_dx)*self.weight_tm/self.num_nodes
            
            grad[-self.num_par:] += 2*np.dot(tor, dt_dp)*self.weight_tm/self.num_nodes
            
        return grad

		
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        N = self.num_nodes
        S = self.num_states
        C = self.num_cons
        h = self.interval

        vs = self.vs_meas

        cons = np.zeros(C*(N - 1))
        uo = x[self.i_openT[-1, -1]+1:self.i_openT[-1, -1]+1+self.num_normal*self.num_control]
        con = x[-self.num_par:]
        
        for p in range(N-1):
			
            x_p = x[p*S : (p+1)*S]
            x_a = x[(p+1)*S : (p+2)*S]
            x_meas_a = self.x_normal_ref[p+1, :]

            vs_a = vs[(p+1),:]
            
            u = x[self.i_openT[:, p+1]]
                        
            index_open = self.index_normal[p+1, :]
            index_imp = self.index_impedance[p+1, :]

            f, _, _, _, _, _ = self.gait2dpi_u(x_a, (x_a - x_p)/h,
                                            x_meas_a, vs_a, u, uo, con,
                                            index_open, index_imp)
					
            cons[C*p : C*(p+1)] = f

        return cons
    
	
    def jacobianstructure(self):
        # Assume the sturcuture of Jac will not change in each iteration
		# (approaved by other project), sparsity functions are used to get
		# the structure of Jac.
		
		# random initial guess is used to get the jacobian and its structure, 
		# Theoritically, the sturcuture can be get by one time only.
		
        N = self.num_nodes
        P = self.num_par
        h = self.interval
        S = self.num_states
        C = self.num_cons
        Nor = self.num_normal
		
        np.random.seed()
        vs = np.random.random((N,2))
		
        np.random.seed()
        x = np.random.random(N*S + N*self.num_open + Nor*self.num_control + P)
        uo = x[self.i_openT[-1, -1]+1:self.i_openT[-1, -1]+1+self.num_normal*self.num_control]
        con = x[-P:]
		
        Jac_x = np.zeros((C, 2*S))
		
        row = np.array([])
        col = np.array([])

        for p in range(N-1):
            
            x_p = x[p*S : (p+1)*S]
            x_a = x[(p+1)*S : (p+2)*S]
            x_meas_a = self.x_normal_ref[p+1, :]
            
            vs_a = vs[(p+1),:]
            
            u = x[self.i_openT[:, p+1]]
            
            index_open = self.index_normal[p+1, :]
            index_imp = self.index_impedance[p+1, :]
 
            _, dfdx, dfdxdot, Jac_u, Jac_uo, Jac_p = self.gait2dpi_u(x_a, (x_a - x_p)/h,
                                                         x_meas_a, vs_a, u, uo, con,
                                                         index_open, index_imp)
					
            # force row 10 col 9 element in dfdx be zero, since it 
            # sometimes is zero, but sometimes is very small number (10^-10).
            # This caused jacobian structure unstable. Thereforce its element
            # is force to be zero here.
            
            dfdx[10, 9] = 0.0
            
            Jac_x[:, :S] =  -dfdxdot/h
            Jac_x[:, S:2*S] = dfdx + dfdxdot/h
            
            for k in range(C):
                row_x, col_x, _ = find(Jac_x[k, :])
                row_u, col_u, _ = find(Jac_u[k, :])
                row_uo, col_uo, _ = find(Jac_uo[k, :])
                row_p, col_p, _ = find(Jac_p[k, :])
                
                row_xf = row_x + p*C + k
                row_uf = row_u + p*C + k
                row_uof = row_uo + p*C + k
                row_pf = row_p + p*C + k
                                
                col_xf = col_x + p*S
                col_uf = col_u + N*S + (p+1)*self.num_open
                col_uof = col_uo + N*S + self.num_open*self.num_nodes
                col_pf = col_p + N*S + self.num_open*self.num_nodes + self.num_control*self.num_normal
					
                row = np.hstack((row, np.hstack((row_xf, row_uf, row_uof, row_pf))))
                col = np.hstack((col, np.hstack((col_xf, col_uf, col_uof, col_pf))))
            
        return (row, col)

		
    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
		
        N = self.num_nodes
        h = self.interval
        S = self.num_states
        C = self.num_cons
        P = self.num_par
        
        vs = self.vs_meas
        uo = x[self.i_openT[-1, -1]+1:self.i_openT[-1, -1]+1+self.num_normal*self.num_control]
        con = x[-P:]
                
        Jac = np.array([])
        Jac_p = np.zeros((C, 2*S + self.num_open + self.num_control*self.num_normal + P))

        for p in range(N-1):
			
            x_p = x[p*S : (p+1)*S]
            x_a = x[(p+1)*S : (p+2)*S]
            vs_a = vs[(p+1),:]
            
            x_meas_a = self.x_normal_ref[p+1, :]
            
            u = x[self.i_openT[:, p+1]]
            
            index_open = self.index_normal[p+1, :]
            index_imp = self.index_impedance[p+1, :]

            _, dfdx, dfdxdot, dfdu, dfduo, dfdp = self.gait2dpi_u(x_a, (x_a - x_p)/h,
                                                           x_meas_a, vs_a, u, uo,
                                                           con, index_open,
                                                           index_imp)
            
			# force row 10 col 9 element in dfdx be zero, since it 
            # sometimes is zero, but sometimes is very small number (10^-10).
            # This caused jacobian structure unstable. Thereforce its element
            # is force to be zero here.
            
            dfdx[10, 9] = 0.0
                
            Jac_p[:, :S] = -dfdxdot/h
            Jac_p[:, S:2*S] = dfdx + dfdxdot/h
            Jac_p[:, 2*S:2*S + self.num_open] = dfdu
            Jac_p[:, 2*S + self.num_open:2*S + self.num_open + self.num_control*self.num_normal] = dfduo
            Jac_p[:, -P:] = dfdp

            for r in range(C):
                row, col, RA_x = find(Jac_p[r, :])
                Jac = np.hstack((Jac, RA_x))
				
        return Jac
        
    
    def gait2dpi_u(self, xs, xsd, xs_meas, vs, u, uo, p, index_open, index_imp):
        
        half_state = int(self.num_states/2)
        x = xs[:half_state]
        xd = xs[half_state:]
        xdd = xsd[half_state:]
        
        QQ, dQQ_dx, dQQ_dxd, dQQ_dxdd, GRF, dGRF_dx, dGRF_dxd, sticks = \
                                    autolev_rhs(x, xd, xdd, vs, self.constants_dict)

        tor_imp, dtor_dx, dtor_dp = self.impedance_control(xs, xs_meas, p, index_imp)
        
        tor_nor, dtor_duo = self.normal_torque(uo, index_open) 

        tor = np.zeros(half_state)
        
        tor[self.index_open_motion] = u
        tor[self.index_control_motion] = tor_nor + tor_imp
        
        f = np.zeros(self.num_states)
        f[:half_state] = xsd[:half_state] - xs[half_state:]
        f[half_state:] = (tor - QQ)/self.scaling
         
        dfdx = np.zeros((self.num_states, self.num_states))
        dfdx[:half_state,half_state:] = -np.eye(half_state)
        dfdx[half_state:,:half_state] = - np.reshape(dQQ_dx, (half_state, half_state))/self.scaling
        dfdx[half_state:,half_state:] = - np.reshape(dQQ_dxd, (half_state, half_state))/self.scaling
        dfdx[half_state+self.index_control_motion,:] += dtor_dx/self.scaling
        
        dfdxd = np.zeros((self.num_states, self.num_states))
        dfdxd[:half_state,:half_state] = np.eye(half_state)
        dfdxd[half_state:,half_state:] = -np.reshape(dQQ_dxdd, (half_state, half_state))/self.scaling
             
        dfdu = np.zeros((self.num_states, self.num_open))
        dfdu[half_state + self.index_open_motion, :] = np.eye(self.num_open)/self.scaling
        
        dfduo = np.zeros((self.num_states, self.num_normal*self.num_control))
        dfduo[half_state + self.index_control_motion, :] = dtor_duo/self.scaling
        
        dfdp = np.zeros((self.num_states, self.num_par))
        dfdp[half_state + self.index_control_motion, :] = dtor_dp/self.scaling
        
        return f, dfdx, dfdxd, dfdu, dfduo, dfdp
    
    def impedance_control(self, xs, xs_meas, p, index):
        
        tor = np.zeros(self.num_control)
        dtor_dx = np.zeros((self.num_control, self.num_states))
        dtor_dp = np.zeros((self.num_control, self.num_par))
        
        par_joint = int(self.num_par/self.num_control)
        
        for i in range(self.num_control):
            if self.index_control_motion[i] < 6:
                tor[i] = (p[i*par_joint + int(index[0]-1)*2]*(xs_meas[i] 
                                                                - xs[self.index_control_motion[i]]) 
                         + p[i*par_joint + int(index[0]-1)*2+1]*(xs_meas[self.num_control + i] 
                                                                - xs[self.index_control_motion[i]+9]))
                
                dtor_dx[i, self.index_control_motion[i]] = -p[i*par_joint + int(index[0]-1)*2]
                dtor_dx[i, self.index_control_motion[i]+9] = -p[i*par_joint + int(index[0]-1)*2+1]
                
                dtor_dp[i, i*par_joint + int(index[0]-1)*2] += xs_meas[i] - xs[self.index_control_motion[i]]
                dtor_dp[i, i*par_joint + int(index[0]-1)*2+1] += xs_meas[self.num_control + i] - xs[self.index_control_motion[i]+9]
                
            else:
                tor[i] = (p[i*par_joint + int(index[1]-1)*2]*(xs_meas[i] 
                                                                - xs[self.index_control_motion[i]]) 
                         + p[i*par_joint + int(index[1]-1)*2+1]*(xs_meas[self.num_control + i] 
                                                                - xs[self.index_control_motion[i]+9]))
                
                dtor_dx[i, self.index_control_motion[i]] = -p[i*par_joint + int(index[1]-1)*2]
                dtor_dx[i, self.index_control_motion[i]+9] = -p[i*par_joint + int(index[1]-1)*2+1]
                
                dtor_dp[i, i*par_joint + int(index[1]-1)*2] += xs_meas[i] - xs[self.index_control_motion[i]]
                dtor_dp[i, i*par_joint + int(index[1]-1)*2+1] += xs_meas[self.num_control + i] - xs[self.index_control_motion[i]+9]
            
        return tor, dtor_dx, dtor_dp
    
    def normal_torque(self, uo, index_open):
        
        tor = np.zeros(self.num_control)
        dtor_duo = np.zeros((self.num_control, self.num_normal*self.num_control))
        
        i_normalT_local = self.i_normalT - self.i_openT[-1, -1] -1

        for i in range(self.num_control):
            
            if self.index_control_motion[i] < 6:
                
                if index_open[0] == 49:
                    
                    tor[i] = uo[i_normalT_local[i, int(index_open[0])]]
                    
                    dtor_duo[i, i_normalT_local[i, int(index_open[0])]] = 1
                    
                else:
                    
                    tor[i] = ((-index_open[0] + 1 + int(index_open[0]))*uo[i_normalT_local[i, int(index_open[0])]] + 
                         (index_open[0] - int(index_open[0]))*uo[i_normalT_local[i, int(index_open[0])+1]])
                    
                    dtor_duo[i, i_normalT_local[i, int(index_open[0])]] = (-index_open[0] + 1 + int(index_open[0]))
                    dtor_duo[i, i_normalT_local[i, int(index_open[0]) + 1]] = (index_open[0] - int(index_open[0]))
                    
            else:
                
                if index_open[1] == 49:
                    
                    tor[i] = uo[i, i_normalT_local[i, int(index_open[1])]]
                    
                    dtor_duo[i_normalT_local[i, int(index_open[1])]] = 1
                else:
                    tor[i] = ((-index_open[1] +1 + int(index_open[1]))*uo[i_normalT_local[i, int(index_open[1])]] + 
                         (index_open[1] - int(index_open[1]))*uo[i_normalT_local[i, int(index_open[1])+1]])
                    
                    dtor_duo[i, i_normalT_local[i, int(index_open[1])]] = (-index_open[1] +1 + int(index_open[1]))
                    dtor_duo[i, i_normalT_local[i, int(index_open[1])+1]] = (index_open[1] - int(index_open[1]))
                    
        return tor, dtor_duo
                    
                    
 
        
        
