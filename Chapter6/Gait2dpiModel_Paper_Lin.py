#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 10:46:43 2017

Revised on Wed May 02 12:13:00 2018

@author: huawei

This program defined gait2dpi model for direct collocation.
"""

import sys
sys.path.append('../')

import numpy as np
from gait2dpi_dGRF_LinContact import evaluate_autolev_rhs as autolev_rhs
from simulate_auto import map_values_to_autolev_symbols
from scipy.sparse import find
from numpy import sin, cos

class gait2dpi_model(object):
    def __init__(self, x_meas, vs_meas, initial_L, initial_R, num_nodes, num_states,
                 num_con_close, num_stanceL, num_stanceR, num_ankle, coe_swingx,
                 coe_swingy, swingy_endp, stancex_endp, constants_dict,
                 interval, lamda, scaling =1):
        
        self.x_meas = x_meas
        self.vs_meas = vs_meas
        self.num_nodes = num_nodes
        self.num_states = num_states
        self.num_cons = num_states
        self.num_con_close = num_con_close
        self.num_stanceL = num_stanceL
        self.num_stanceR = num_stanceR
        self.num_ankle = num_ankle
        self.num_conspernode = num_states
        self.coe_swingx = coe_swingx
        self.coe_swingy = coe_swingy
        self.swingy_endp = swingy_endp
        self.stancex_endp = stancex_endp
        
        self.num_high_con = 2

        self.interval = interval
        self.scaling = scaling
        
        self.lamda = lamda
        
        self.initial_L = initial_L
        self.initial_R = initial_R
        
        self.z0 = 0.83
        self.g = 9.81
        self.omega = np.sqrt(self.g/self.z0)
        
        self.index = np.linspace(0, self.num_states*self.num_nodes,
                                  self.num_nodes, endpoint=False, dtype=int)
        
        self.i_x = self.index
        self.i_y = self.index + 1
        self.i_tors = self.index + 2
        self.i_LHip = self.index + 3
        self.i_LKnee = self.index + 4
        self.i_LAnkle = self.index + 5
        self.i_RHip = self.index + 6
        self.i_RKnee = self.index + 7
        self.i_RAnkle = self.index + 8
        
        self.i_HipRef = self.index + 18
        self.i_KneeRef = self.index + 19
        
        self.constants_dict = map_values_to_autolev_symbols(constants_dict)
        
        
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        
        weight = 0.0001
        
#        f_x = np.sum((x[self.i_x] - self.x_meas[self.i_x])**2)
        #f_y = np.sum((x[self.i_y] - self.x_meas[self.i_y])**2)
        f_tors = np.sum((x[self.i_tors] - self.x_meas[self.i_tors])**2)       
        f_Hip = np.sum((x[self.i_RHip] - self.x_meas[self.i_RHip])**2) + np.sum((x[self.i_LHip] - self.x_meas[self.i_LHip])**2)
        f_Knee = np.sum((x[self.i_RKnee] - self.x_meas[self.i_RKnee])**2) + np.sum((x[self.i_LKnee] - self.x_meas[self.i_LKnee])**2)
        f_Ankle = np.sum((x[self.i_RAnkle] - self.x_meas[self.i_RAnkle])**2) + np.sum((x[self.i_LAnkle] - self.x_meas[self.i_LAnkle])**2)
        
        f_traj = (f_tors + f_Hip + f_Knee + f_Ankle)
        
        #f_gains = weight*np.sum(x[self.num_states*self.num_nodes+2:self.num_states*self.num_nodes+self.num_con_close]**2)
        
        f_torque = weight*np.sum(x[self.num_states*self.num_nodes+self.num_con_close:]**2)
        
        obj = 10*(self.lamda*f_traj + (1-self.lamda)*(f_torque))/self.num_nodes
        
        return obj
    
	 
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        
        weight = 0.0001
        
        grad = np.zeros_like(x)
        
#        grad[self.i_x] = 2.0/self.num_nodes*self.lamda*(x[self.i_x] - self.x_meas[self.i_x])*10
        #grad[self.i_y] = 2.0/self.num_nodes*self.lamda*(x[self.i_y] - self.x_meas[self.i_y])*100
        grad[self.i_tors] = 2.0/self.num_nodes*self.lamda*(x[self.i_tors] - self.x_meas[self.i_tors])*10
        
        grad[self.i_RHip] = 2.0/self.num_nodes*self.lamda*(x[self.i_RHip] - self.x_meas[self.i_RHip])*10
        grad[self.i_LHip] = 2.0/self.num_nodes*self.lamda*(x[self.i_LHip] - self.x_meas[self.i_LHip])*10
        
        grad[self.i_RKnee] = 2.0/self.num_nodes*self.lamda*(x[self.i_RKnee] - self.x_meas[self.i_RKnee])*10
        grad[self.i_LKnee] = 2.0/self.num_nodes*self.lamda*(x[self.i_LKnee] - self.x_meas[self.i_LKnee])*10
        
        grad[self.i_RAnkle] = 2.0/self.num_nodes*self.lamda*(x[self.i_RAnkle] - self.x_meas[self.i_RAnkle])*10
        grad[self.i_LAnkle] = 2.0/self.num_nodes*self.lamda*(x[self.i_LAnkle] - self.x_meas[self.i_LAnkle])*10
        
#        grad[self.num_states*self.num_nodes+2:self.num_states*self.num_nodes+self.num_con_close] =\
#        2.0*weight*(1-self.lamda)/self.num_nodes*x[self.num_states*self.num_nodes+2:self.num_states*self.num_nodes+self.num_con_close]*100

        grad[self.num_states*self.num_nodes+self.num_con_close:] = 2.0*weight*(1-self.lamda)/self.num_nodes*x[self.num_states*self.num_nodes+self.num_con_close:]*10
        
        return grad
		
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        
        L1 = self.num_nodes*self.num_states + self.num_con_close
        L2 = L1 + self.num_stanceL*2 + self.num_stanceR*2
        
        con_close = x[self.num_nodes*self.num_states : L1]
        con_stanceL = x[L1 : L1 + self.num_stanceL*2]
        con_stanceR = x[L1 + self.num_stanceL*2 : L2]
        con_ankle = x[L2 : L2 + (self.num_nodes-1)*2]
        
        cons = np.array([])
        
        k = 0
        j = 0

        for p in range(self.num_nodes-1):
			
            x_p = x[p*self.num_states : (p+1)*self.num_states]
            x_a = x[(p+1)*self.num_states : (p+2)*self.num_states]
				
            vs_a = self.vs_meas[(p+1),:]
            
            if self.initial_L[p+1, 0]:
                u_stanceL = con_stanceL[k*2 : (k+1)*2]
                k += 1
            else:
                u_stanceL = np.array([])
                
            if self.initial_R[p+1, 0]:
                u_stanceR = con_stanceR[j*2 : (j+1)*2]
                j += 1
            else:
                u_stanceR = np.array([])
            
            u_ankle = con_ankle[p*2 : (p+1)*2]
            
            iniL = self.initial_L[p+1, :]
            iniR = self.initial_R[p+1, :]
				
            f, df_dXs, df_dXsd, df_dConClose, df_dConLOpen, df_dConROpen, df_dConAnkle = \
            self.gait2dpi_u(x_a, (x_a - x_p)/self.interval, vs_a, con_close,
                            u_stanceL, u_stanceR, u_ankle, iniL, iniR)
            
            cons = np.hstack((cons, f))

        return cons
    
	
    def jacobianstructure(self):
        # Assume the sturcuture of Jac will not change in each iteration
		# (approaved by other project), sparsity functions are used to get
		# the structure of Jac.
		
		# random initial guess is used to get the jacobian and its structure, 
		# Theoritically, the sturcuture can be get by one time only.
        
        num_test = 3
    
        row = np.array([])
        col = np.array([])
        
        k = 0
        j = 0

        for p in range(self.num_nodes-1):
            Jac_x = np.zeros((self.num_cons, 2*self.num_states))
            Jac_con_close = np.zeros((self.num_cons, self.num_con_close))
            Jac_con_Lopen = np.zeros((self.num_cons, 2))
            Jac_con_Ropen = np.zeros((self.num_cons, 2))
            Jac_con_ankle = np.zeros((self.num_cons, 2))
            
            iniL = self.initial_L[p+1, :]
            iniR = self.initial_R[p+1, :]
        
            for q in range(num_test):
                np.random.seed()
                x_p = 0.5 - np.random.random(self.num_states)
                np.random.seed()
                x_a = 0.5 - np.random.random(self.num_states)
                np.random.seed()
                vs_a = 0.1*np.random.random(2)
                np.random.seed()
                con_close = 1 -2*np.random.random(self.num_con_close)
                np.random.seed()
                u_ankle = 100*np.random.random(2)
                
                np.random.seed()
                u_stanceL = 100*np.random.random(2)
                np.random.seed()
                u_stanceR = 100*np.random.random(2)
                
                f, dfdx, dfdxdot, df_dConClose, df_dConLOpen, df_dConROpen, df_dConAnkle =\
                self.gait2dpi_u(x_a, (x_a - x_p)/self.interval, vs_a, con_close,
                            u_stanceL, u_stanceR, u_ankle, iniL, iniR)
                
                Jac_x[:, :self.num_states] -= dfdxdot/self.interval
                
                Jac_x[:, self.num_states:2*self.num_states] +=\
                (dfdx + dfdxdot/self.interval)
                
                Jac_con_close += df_dConClose
                Jac_con_Lopen += df_dConLOpen
                Jac_con_Ropen += df_dConROpen
                Jac_con_ankle += df_dConAnkle
                
            for r in range(self.num_cons):
                
                row_x, col_x, RA_Jac_x = find(Jac_x[r, :])
                row_c, col_c, RA_Jac_c = find(Jac_con_close[r, :])
                row_lo, col_lo, RA_Jac_lo = find(Jac_con_Lopen[r, :])
                row_ro, col_ro, RA_Jac_ro = find(Jac_con_Ropen[r, :])
                row_a, col_a, RA_Jac_a = find(Jac_con_ankle[r, :])

                
                row_xf = row_x + p*self.num_cons + r
                row_cf = row_c + p*self.num_cons + r
                row_lof = row_lo + p*self.num_cons + r
                row_rof = row_ro + p*self.num_cons + r
                row_af = row_a + p*self.num_cons + r
                
                col_xf = col_x + p*self.num_states
                col_cf = col_c + self.num_nodes*self.num_states
                col_lof = col_lo + self.num_nodes*self.num_states + self.num_con_close + k*2
                col_rof = col_ro + self.num_nodes*self.num_states + self.num_con_close + self.num_stanceL*2 + j*2
                col_af = col_a + self.num_nodes*self.num_states + self.num_con_close + self.num_stanceL*2 + self.num_stanceR*2 + p*2
					
                row = np.hstack((row, row_xf, row_cf, row_lof, row_rof, row_af))
                col = np.hstack((col, col_xf, col_cf, col_lof, col_rof, col_af))
                    
            if iniL[0]:
                k += 1
            if iniR[0]:
                j += 1
		
        return (row, col)

		
    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        L1 = self.num_nodes*self.num_states + self.num_con_close
        L2 = L1 + self.num_stanceL*2 + self.num_stanceR*2
        
        con_close = x[self.num_nodes*self.num_states : L1]
        con_stanceL = x[L1 : L1 + self.num_stanceL*2]
        con_stanceR = x[L1 + self.num_stanceL*2 : L2]
        con_ankle = x[L2 : L2 + (self.num_nodes-1)*2]
        
        Jac = np.array([])
        
        k = 0
        j = 0

        for p in range(self.num_nodes-1):
                        
            Jac_full = np.zeros((self.num_cons, 2*self.num_states+self.num_con_close + 3*2))
            
            x_p = x[p*self.num_states : (p+1)*self.num_states]
            x_a = x[(p+1)*self.num_states : (p+2)*self.num_states]
				
            vs_a = self.vs_meas[(p+1),:]
            
            u_ankle = con_ankle[p*2 : (p+1)*2]
            
            iniL = self.initial_L[p+1, :]
            iniR = self.initial_R[p+1, :]
            
            if self.initial_L[p+1, 0] and self.initial_L[p+1, 0]:
                u_stanceL = con_stanceL[k*2 : (k+1)*2]
                k += 1
            else:
                u_stanceL = np.array([])
                
            if self.initial_R[p+1, 0]:
                u_stanceR = con_stanceR[j*2 : (j+1)*2]
                j += 1
            else:
                u_stanceR = np.array([])
            

            f, dfdx, dfdxdot, df_dConClose, df_dConLOpen, df_dConROpen, df_dConAnkle =\
                self.gait2dpi_u(x_a, (x_a - x_p)/self.interval, vs_a, con_close,
                            u_stanceL, u_stanceR, u_ankle, iniL, iniR)

            Jac_full[:, :self.num_states] = -dfdxdot/self.interval
            Jac_full[:, self.num_states:2*self.num_states] = dfdx + dfdxdot/self.interval
            Jac_full[:, 2*self.num_states:2*self.num_states+self.num_con_close] = df_dConClose
            Jac_full[:, 2*self.num_states+self.num_con_close:2*self.num_states+self.num_con_close+2] = df_dConLOpen
            Jac_full[:, 2*self.num_states+self.num_con_close+2:2*self.num_states+self.num_con_close+4] = df_dConROpen
            Jac_full[:, 2*self.num_states+self.num_con_close+4:2*self.num_states+self.num_con_close+6] = df_dConAnkle

            for r in range(self.num_cons):                
                row, col, RA_x = find(Jac_full[r, :])
                Jac = np.hstack((Jac, RA_x))
				
        return Jac    


#    def jacobian(self, x):
#        #
#        # The callback for calculating the Jacobian
#        #
#        L1 = self.num_nodes*self.num_states + self.num_con_close
#        L2 = L1 + self.num_stanceL*2 + self.num_stanceR*2
#        
#        con_close = x[self.num_nodes*self.num_states : L1]
#        con_stanceL = x[L1 : L1 + self.num_stanceL*2]
#        con_stanceR = x[L1 + self.num_stanceL*2 : L2]
#        con_ankle = x[L2 : L2 + (self.num_nodes-1)*2]
#        
#        Jac = np.array([])
#        
#        self.rowJ = np.array([])
#        self.colJ = np.array([])
#        
#
#        
#        k = 0
#        j = 0
#
#        for p in range(self.num_nodes-1):
#            
#            Jac_x = np.zeros((self.num_cons, 2*self.num_states))
#            Jac_con_close = np.zeros((self.num_cons, self.num_con_close))
#            Jac_con_Lopen = np.zeros((self.num_cons, 2))
#            Jac_con_Ropen = np.zeros((self.num_cons, 2))
#            Jac_con_ankle = np.zeros((self.num_cons, 2))
#            
#            x_p = x[p*self.num_states : (p+1)*self.num_states]
#            x_a = x[(p+1)*self.num_states : (p+2)*self.num_states]
#				
#            vs_a = self.vs_meas[(p+1),:]
#            
#            u_ankle = con_ankle[p*2 : (p+1)*2]
#            
#            iniL = self.initial_L[p+1, :]
#            iniR = self.initial_R[p+1, :]
#            
#            if self.initial_L[p+1, 0] and self.initial_L[p+1, 0]:
#                u_stanceL = con_stanceL[k*2 : (k+1)*2]
#            else:
#                u_stanceL = np.array([])
#                
#            if self.initial_R[p+1, 0]:
#                u_stanceR = con_stanceR[j*2 : (j+1)*2]
#            else:
#                u_stanceR = np.array([])
#            
#
#            f, dfdx, dfdxdot, df_dConClose, df_dConLOpen, df_dConROpen, df_dConAnkle =\
#                self.gait2dpi_u(x_a, (x_a - x_p)/self.interval, vs_a, con_close,
#                            u_stanceL, u_stanceR, u_ankle, iniL, iniR)
#
#            Jac_x[:, :self.num_states] -= dfdxdot/self.interval
#            
#            Jac_x[:, self.num_states:2*self.num_states] +=\
#            (dfdx + dfdxdot/self.interval)
#            
#            Jac_con_close += df_dConClose
#            Jac_con_Lopen += df_dConLOpen
#            Jac_con_Ropen += df_dConROpen
#            Jac_con_ankle += df_dConAnkle
#
#            for r in range(self.num_cons):
#                
#                row_x, col_x, RA_Jac_x = find(Jac_x[r, :])
#                row_c, col_c, RA_Jac_c = find(Jac_con_close[r, :])
#                row_lo, col_lo, RA_Jac_lo = find(Jac_con_Lopen[r, :])
#                row_ro, col_ro, RA_Jac_ro = find(Jac_con_Ropen[r, :])
#                row_a, col_a, RA_Jac_a = find(Jac_con_ankle[r, :])
#
#                
#                row_xf = row_x + p*self.num_cons + r
#                row_cf = row_c + p*self.num_cons + r
#                row_lof = row_lo + p*self.num_cons + r
#                row_rof = row_ro + p*self.num_cons + r
#                row_af = row_a + p*self.num_cons + r
#                
#                col_xf = col_x + p*self.num_states
#                col_cf = col_c + self.num_nodes*self.num_states
#                col_lof = col_lo + self.num_nodes*self.num_states + self.num_con_close + k*2
#                col_rof = col_ro + self.num_nodes*self.num_states + self.num_con_close + self.num_stanceL*2 + j*2
#                col_af = col_a + self.num_nodes*self.num_states + self.num_con_close + self.num_stanceL*2 + self.num_stanceR*2 + p*2
#					
#                self.rowJ = np.hstack((self.rowJ, row_xf, row_cf, row_lof, row_rof, row_af))
#                self.colJ = np.hstack((self.colJ, col_xf, col_cf, col_lof, col_rof, col_af))
#                Jac = np.hstack((Jac, RA_Jac_x, RA_Jac_c, RA_Jac_lo, RA_Jac_ro, RA_Jac_a))
#                    
#            if iniL[0]:
#                k += 1
#            if iniR[0]:
#                j += 1
#                
#        return Jac
        

        
    
    def high_swing_con_R(self, xs, con_swing, con_exp, time_vector, start_swing):
        
        Rthigh = self.constants_dict['ThighLen']
        Rshank = self.constants_dict['ShankLen']
        
        trunk_relative_x = -Rthigh*sin(xs[2] + xs[3]) - Rshank*sin(xs[2] + xs[3] + xs[4])
        
        trunk_relative_xd = (-Rthigh*cos(xs[2] + xs[3])*xs[11] - Rthigh*cos(xs[2] + xs[3])*xs[12]
                            - Rshank*cos(xs[2] + xs[3] + xs[4])*xs[11] - Rshank*cos(xs[2] + xs[3] + xs[4])*xs[12]
                            - Rshank*cos(xs[2] + xs[3] + xs[4])*xs[13])
        
        dtrunk_dxs = np.zeros((2, self.num_states))
        dtrunk_dxs[0, 2] = -Rthigh*cos(xs[2] + xs[3]) - Rshank*cos(xs[2] + xs[3] + xs[4])
        dtrunk_dxs[0, 3] = -Rthigh*cos(xs[2] + xs[3]) - Rshank*cos(xs[2] + xs[3] + xs[4])
        dtrunk_dxs[0, 4] = -Rshank*cos(xs[2] + xs[3] + xs[4])
        
        dtrunk_dxs[1, 2] = (Rthigh*sin(xs[2] + xs[3])*xs[11] + Rthigh*sin(xs[2] + xs[3])*xs[12]
                            + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[11] + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[12]
                            + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[13])
        
        dtrunk_dxs[1, 3] = (Rthigh*sin(xs[2] + xs[3])*xs[11] + Rthigh*sin(xs[2] + xs[3])*xs[12]
                            + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[11] + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[12]
                            + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[13])
        
        dtrunk_dxs[1, 4] = (Rshank*sin(xs[2] + xs[3] + xs[4])*xs[11] + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[12]
                            + Rshank*sin(xs[2] + xs[3] + xs[4])*xs[13])
        
        dtrunk_dxs[1, 11] = (-Rthigh*cos(xs[2] + xs[3]) - Rshank*cos(xs[2] + xs[3] + xs[4]))
        
        dtrunk_dxs[1, 12] = (-Rthigh*cos(xs[2] + xs[3]) - Rshank*cos(xs[2] + xs[3] + xs[4]))
        
        dtrunk_dxs[1, 13] = (-Rshank*cos(xs[2] + xs[3] + xs[4]))


        predict_swing = np.zeros(2)
        dPredictSwing_dCon = np.zeros((2, len(con_swing)))
        dPredictSwing_dxs = np.zeros((2, self.num_states))
        dPredictSwing_dConExp = np.zeros((2, 1))
    
        predict_swingx0 = con_swing[0]*trunk_relative_x + con_swing[1]*trunk_relative_xd/self.omega
        
        predict_swing[0] = (predict_swingx0)*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0])) - self.stancex_endp
        predict_swing[1] = self.swingy_endp
        
        dPredictSwing_dxs[0, :] = ((con_swing[0]*dtrunk_dxs[0, :] + con_swing[1]*dtrunk_dxs[1, :]/self.omega)*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0])))
        
        dPredictSwing_dCon[0, 0] = trunk_relative_x*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0]))
        dPredictSwing_dCon[0, 1] = trunk_relative_xd/self.omega*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0]))
        
        #dPredictSwing_dConExp[0, 0] = (predict_swingx0)*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0]))*(self.omega*(time_vector[1]-time_vector[0]))
        
        return predict_swing, dPredictSwing_dxs, dPredictSwing_dCon, dPredictSwing_dConExp
    
    
    def high_swing_con_L(self, xs, con_swing, con_exp, time_vector, start_swing):
        
        Rthigh = self.constants_dict['ThighLen']
        Rshank = self.constants_dict['ShankLen']
        
        trunk_relative_x = -Rthigh*sin(xs[2] + xs[6]) - Rshank*sin(xs[2] + xs[6] + xs[7])
        
        trunk_relative_xd = (-Rthigh*cos(xs[2] + xs[6])*xs[11] - Rthigh*cos(xs[2] + xs[6])*xs[15]
                            - Rshank*cos(xs[2] + xs[6] + xs[7])*xs[11] - Rshank*cos(xs[2] + xs[6] + xs[7])*xs[15]
                            - Rshank*cos(xs[2] + xs[6] + xs[7])*xs[16])
                            
        dtrunk_dxs = np.zeros((2, self.num_states))
        
        dtrunk_dxs[0, 2] = -Rthigh*cos(xs[2] + xs[6]) - Rshank*cos(xs[2] + xs[6] + xs[7])
        dtrunk_dxs[0, 6] = -Rthigh*cos(xs[2] + xs[6]) - Rshank*cos(xs[2] + xs[6] + xs[7])
        dtrunk_dxs[0, 7] = -Rshank*cos(xs[2] + xs[6] + xs[7])
        
        dtrunk_dxs[1, 2] = (Rthigh*sin(xs[2] + xs[6])*xs[11] + Rthigh*sin(xs[2] + xs[6])*xs[15]
                            + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[11] + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[15]
                            + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[16])
        
        dtrunk_dxs[1, 6] = (Rthigh*sin(xs[2] + xs[6])*xs[11] + Rthigh*sin(xs[2] + xs[6])*xs[15]
                            + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[11] + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[15]
                            + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[16])
        
        dtrunk_dxs[1, 7] = (Rshank*sin(xs[2] + xs[6] + xs[7])*xs[11] + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[15]
                            + Rshank*sin(xs[2] + xs[6] + xs[7])*xs[16])
        
        dtrunk_dxs[1, 11] = (-Rthigh*cos(xs[2] + xs[6]) - Rshank*cos(xs[2] + xs[6] + xs[7]))
        
        dtrunk_dxs[1, 15] = (-Rthigh*cos(xs[2] + xs[6]) - Rshank*cos(xs[2] + xs[6] + xs[7]))
        
        dtrunk_dxs[1, 16] = (-Rshank*cos(xs[2] + xs[6] + xs[7]))
        
        predict_swing = np.zeros(2)
        dPredictSwing_dCon = np.zeros((2, len(con_swing)))
        dPredictSwing_dxs = np.zeros((2, self.num_states))
        dPredictSwing_dConExp = np.zeros((2, 1))
    
        predict_swingx0 = con_swing[0]*trunk_relative_x + con_swing[1]*trunk_relative_xd/self.omega
        
        predict_swing[0] = (predict_swingx0)*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0])) - self.stancex_endp
        predict_swing[1] = self.swingy_endp
        
        dPredictSwing_dxs[0, :] = ((con_swing[0]*dtrunk_dxs[0, :] + con_swing[1]*dtrunk_dxs[1, :]/self.omega)*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0])))
        
        dPredictSwing_dCon[0, 0] = trunk_relative_x*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0]))
        dPredictSwing_dCon[0, 1] = trunk_relative_xd/self.omega*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0]))
        
        #dPredictSwing_dConExp[0, 0] = (predict_swingx0)*np.exp(self.omega*con_exp*(time_vector[1]-time_vector[0]))*(self.omega*(time_vector[1]-time_vector[0]))
        
        return predict_swing, dPredictSwing_dxs, dPredictSwing_dCon, dPredictSwing_dConExp
    
    def swing_current_point(self, swing0, time, predict):
         
        swing_t = time[0]
        swing_tp = time[1]
         
        B = np.array([self.coe_swingx[0], self.coe_swingy[0]])
        C = np.array([self.coe_swingx[1], self.coe_swingy[1]])
        D = np.array([self.coe_swingx[2], self.coe_swingy[2]])
        E = np.array([self.coe_swingx[3], self.coe_swingy[3]])
        F = np.array([self.coe_swingx[4], self.coe_swingy[4]])
        
        Scale = predict - swing0
        ScaleT = np.array([(swing_t)/(swing_tp),
                           (swing_t)/(swing_tp)])
    
        point = (swing0 + B*Scale*ScaleT + C*Scale*ScaleT**2 
                     + D*Scale*ScaleT**3 + E*Scale*ScaleT**4 
                     + F*Scale*ScaleT**5)
        
        point_d = (B*Scale*(1/swing_tp) + 2*C*Scale*ScaleT*(1/swing_tp)
                     + 3*D*Scale*ScaleT**2*(1/swing_tp) + 4*E*Scale*ScaleT**3*(1/swing_tp) 
                     + 5*F*Scale*ScaleT**4*(1/swing_tp))
        
        dPoint_dPredict = np.diag(B*ScaleT + C*ScaleT**2 
                                     + D*ScaleT**3 + E*ScaleT**4 + F*ScaleT**5)
        
        dPointd_dPredict = np.diag((B*(1/swing_tp) + 2*C*ScaleT*(1/swing_tp)
                     + 3*D*ScaleT**2*(1/swing_tp) + 4*E*ScaleT**3*(1/swing_tp) 
                     + 5*F*ScaleT**4*(1/swing_tp)))
        
        points= np.hstack((point, point_d))
        dPoints_dPredict = np.vstack((dPoint_dPredict, dPointd_dPredict))

        return points, dPoints_dPredict
    
    def IK_swingL(self, Swing_points, x_states):
        
        Lthigh = self.constants_dict['ThighLen']
        Lshank = self.constants_dict['ShankLen']
        
        fSwing = np.zeros(4)
        dfdSwing_angs = np.zeros((4, self.num_states))
        
        fSwing[0] = Lthigh*sin(x_states[2] + x_states[18]) + Lshank*sin(x_states[2]+x_states[18]+x_states[19]) - Swing_points[0] 
        fSwing[1] = Lthigh*cos(x_states[2] + x_states[18]) + Lshank*cos(x_states[2]+x_states[18]+x_states[19]) + Swing_points[1]
        
        fSwing[2] = (Lthigh*cos(x_states[2] + x_states[18])*(x_states[11]+x_states[20])
                    + Lshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]) 
                    - Swing_points[2])
        fSwing[3] = (-Lthigh*sin(x_states[2] + x_states[18])*(x_states[11]+x_states[20]) 
                    - Lshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21])
                    + Swing_points[3])
        
        dfdSwing_points = np.diag([-1, 1, -1, 1])
        
        dfdSwing_angs[0, 2] = Lthigh*cos(x_states[2] + x_states[18]) + Lshank*cos(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[0, 18] = Lthigh*cos(x_states[2] + x_states[18]) + Lshank*cos(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[0, 19] = Lshank*cos(x_states[2]+x_states[18]+x_states[19])
        
        dfdSwing_angs[1, 2] = -Lthigh*sin(x_states[2] + x_states[18]) - Lshank*sin(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[1, 18] = -Lthigh*sin(x_states[2] + x_states[18]) - Lshank*sin(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[1, 19] = -Lshank*sin(x_states[2]+x_states[18]+x_states[19])
        
        dfdSwing_angs[2, 2] = (-Lthigh*sin(x_states[2] + x_states[18])*(x_states[11]+x_states[20]) 
                                - Lshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[2, 18] = (-Lthigh*sin(x_states[2] + x_states[18])*(x_states[11]+x_states[20])  
                                - Lshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[2, 19] = -Lshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21])
        
        dfdSwing_angs[2, 11] = (Lthigh*cos(x_states[2] + x_states[18]) + Lshank*cos(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[2, 20] = (Lthigh*cos(x_states[2] + x_states[18]) + Lshank*cos(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[2, 21] = Lshank*cos(x_states[2]+x_states[18]+x_states[19])
        
        
        dfdSwing_angs[3, 2] = (-Lthigh*cos(x_states[2] + x_states[18])*(x_states[11]+x_states[20]) 
                                - Lshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[3, 18] = (-Lthigh*cos(x_states[2] + x_states[18])*(x_states[11]+x_states[20])  
                                - Lshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[3, 19] = -Lshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21])
        
        dfdSwing_angs[3, 11] = (-Lthigh*sin(x_states[2] + x_states[18]) - Lshank*sin(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[3, 20] = (-Lthigh*sin(x_states[2] + x_states[18]) - Lshank*sin(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[3, 21] = -Lshank*sin(x_states[2]+x_states[18]+x_states[19])
        
        return fSwing, dfdSwing_points, dfdSwing_angs
    
    def IK_swingR(self, Swing_points, x_states):
        
        Rthigh = self.constants_dict['ThighLen']
        Rshank = self.constants_dict['ShankLen']
        
        fSwing = np.zeros(4)
        dfdSwing_angs = np.zeros((4, self.num_states))
        
        fSwing[0] = Rthigh*sin(x_states[2] + x_states[18]) + Rshank*sin(x_states[2]+x_states[18]+x_states[19]) - Swing_points[0] 
        fSwing[1] = Rthigh*cos(x_states[2] + x_states[18]) + Rshank*cos(x_states[2]+x_states[18]+x_states[19]) + Swing_points[1]
        fSwing[2] = (Rthigh*cos(x_states[2] + x_states[18])*(x_states[11]+x_states[20])
                    + Rshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]) 
                    - Swing_points[2])
        fSwing[3] = (-Rthigh*sin(x_states[2] + x_states[18])*(x_states[11]+x_states[20]) 
                    - Rshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21])
                    + Swing_points[3])
        
        dfdSwing_points = np.diag([-1, 1, -1, 1])
        
        dfdSwing_angs[0, 2] = Rthigh*cos(x_states[2] + x_states[18]) + Rshank*cos(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[0, 18] = Rthigh*cos(x_states[2] + x_states[18]) + Rshank*cos(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[0, 19] = Rshank*cos(x_states[2]+x_states[18]+x_states[19])
        
        dfdSwing_angs[1, 2] = -Rthigh*sin(x_states[2] + x_states[18]) - Rshank*sin(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[1, 18] = -Rthigh*sin(x_states[2] + x_states[18]) - Rshank*sin(x_states[2]+x_states[18]+x_states[19])
        dfdSwing_angs[1, 19] = -Rshank*sin(x_states[2]+x_states[18]+x_states[19])
        
        dfdSwing_angs[2, 2] = (-Rthigh*sin(x_states[2] + x_states[18])*(x_states[11]+x_states[20]) 
                                - Rshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[2, 18] = (-Rthigh*sin(x_states[2] + x_states[18])*(x_states[11]+x_states[20])  
                                - Rshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[2, 19] = -Rshank*sin(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21])
        
        dfdSwing_angs[2, 11] = (Rthigh*cos(x_states[2] + x_states[18]) + Rshank*cos(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[2, 20] = (Rthigh*cos(x_states[2] + x_states[18]) + Rshank*cos(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[2, 21] = Rshank*cos(x_states[2]+x_states[18]+x_states[19])
        
        
        dfdSwing_angs[3, 2] = (-Rthigh*cos(x_states[2] + x_states[18])*(x_states[11]+x_states[20]) 
                                - Rshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[3, 18] = (-Rthigh*cos(x_states[2] + x_states[18])*(x_states[11]+x_states[20])  
                                - Rshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21]))
        
        dfdSwing_angs[3, 19] = -Rshank*cos(x_states[2]+x_states[18]+x_states[19])*(x_states[11]+x_states[20]+x_states[21])
        
        dfdSwing_angs[3, 11] = (-Rthigh*sin(x_states[2] + x_states[18]) - Rshank*sin(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[3, 20] = (-Rthigh*sin(x_states[2] + x_states[18]) - Rshank*sin(x_states[2]+x_states[18]+x_states[19]))
        dfdSwing_angs[3, 21] = -Rshank*sin(x_states[2]+x_states[18]+x_states[19])
        
        return fSwing, dfdSwing_points, dfdSwing_angs
    
    def local_swing_con(self, swing_ang_ref, swing_ang_current, local_swing_con):

        swing_torque = np.zeros(2)
        dSwingTorque_dAngRef = np.zeros((2, 4))
        dSwingTorque_dAng = np.zeros((2, 4))
        dSwingTorque_dCon = np.zeros((2, 4))
        
        for k in range(2):
            swing_torque[k] = (local_swing_con[k]*(swing_ang_ref[k] - swing_ang_current[k]) 
                            + local_swing_con[k+2]*(swing_ang_ref[k+2] - swing_ang_current[k+2]))
            
            dSwingTorque_dAngRef[k, k] = local_swing_con[k]
            dSwingTorque_dAngRef[k, k+2] = local_swing_con[k+2]
            dSwingTorque_dAng[k, k] = -local_swing_con[k]
            dSwingTorque_dAng[k, k+2] = -local_swing_con[k+2]
            dSwingTorque_dCon[k, k] = (swing_ang_ref[k] - swing_ang_current[k])
            if dSwingTorque_dCon[k, k] == 0:
                dSwingTorque_dCon[k, k] = 1e-6
            dSwingTorque_dCon[k, 2+k] = (swing_ang_ref[k+2] - swing_ang_current[k+2])
            if dSwingTorque_dCon[k, 2+k] == 0:
                dSwingTorque_dCon[k, 2+k] = 1e-6

        return (swing_torque, dSwingTorque_dAngRef, dSwingTorque_dAng, dSwingTorque_dCon)
              
    def swing_control(self, xs, u_close, time_vec, start_swing, sta_left=False, sta_right=False):
        
        # xs is an array which has 22 elements: x(0:9), xd(9:18), xref(18:22)
        
        num_state = self.num_states
        num_cons = 4  # number of constraints introduced by swing controller
        
        num_high_con = self.num_high_con
        num_local_con = 4
        
        # separate high & low controller parameters
        
        con_swing_high = u_close[:num_high_con]
        con_swing_local = u_close[num_high_con:num_high_con+num_local_con]
        con_exp = 1
        
        fSw = np.zeros(num_cons)
        dfSw_dXs = np.zeros((num_cons, num_state))
        dfSw_dCon = np.zeros((num_cons, self.num_con_close))
 
        dSwingTorque_dXs = np.zeros((int(num_cons/2), num_state))
        dSwingTorque_dCon = np.zeros((int(num_cons/2), self.num_con_close))
        
        if sta_left:
            
            swing_ang_current = xs[[3, 4, 12, 13]]
            swing_ang_ref = xs[18:22]
            
            # high level controller
            predict_swing, dPredictSwing_dxs, dPredictSwing_dCon, dPredictSwing_dConExp =\
            self.high_swing_con_L(xs, con_swing_high, con_exp, time_vec, start_swing)
    
            # stance and swing critical point calculation based on stance starting
            # points, current index, and predictions from high level controller
            swing_point, dSwingPoint_dPredict = \
                self.swing_current_point(start_swing, time_vec, predict_swing)
            
            # inverse kinematics of stance/swing leg to get joint angles.
            fSw, dfSw_dSwingPoint, dfSw_dSwingAngs = self.IK_swingL(swing_point, xs)
            
            # torque calculation of local PD controller
            (swing_torque, dSwingTorque_dAngRef, dSwingTorque_dAng, dSwingTorque_dConLow) = \
             self.local_swing_con(swing_ang_ref, swing_ang_current, con_swing_local)
            
            dSwingTorque_dXs[:, 18:22] = dSwingTorque_dAngRef
            dSwingTorque_dXs[:, [3, 4, 12, 13]] = dSwingTorque_dAng
            dSwingTorque_dCon[:, num_high_con:num_high_con+num_local_con] = dSwingTorque_dConLow
            
            dfSw_dXs = np.dot(dfSw_dSwingPoint , np.dot(dSwingPoint_dPredict
                                            , dPredictSwing_dxs))
            dfSw_dXs += dfSw_dSwingAngs
            
            dfSw_dCon[:, :num_high_con] = np.dot(dfSw_dSwingPoint
                                                        , np.dot(dSwingPoint_dPredict
                                                        , dPredictSwing_dCon))
            
            #dfSw_dCon[:, -1:] = np.dot(dfSw_dSwingPoint, np.dot(dSwingPoint_dPredict
            #                                        , dPredictSwing_dConExp))
            
        if sta_right == True:
            
            swing_ang_current = xs[[6, 7, 15, 16]]
            swing_ang_ref = xs[18:22]
            
            # high level controller
            predict_swing, dPredictSwing_dxs, dPredictSwing_dCon, dPredictSwing_dConExp =\
            self.high_swing_con_R(xs, con_swing_high, con_exp, time_vec, start_swing)
    
            # stance and swing critical point calculation based on stance starting
            # points, current index, and predictions from high level controller
            swing_point, dSwingPoint_dPredict = \
                self.swing_current_point(start_swing, time_vec, predict_swing)
            
            # inverse kinematics of stance/swing leg to get joint angles.
            fSw, dfSw_dSwingPoint, dfSw_dSwingAngs = self.IK_swingR(swing_point, xs)
            
            # torque calculation of local PD controller
            (swing_torque, dSwingTorque_dAngRef, dSwingTorque_dAng, dSwingTorque_dConLow) = \
             self.local_swing_con(swing_ang_ref, swing_ang_current, con_swing_local)
            
            dSwingTorque_dXs[:, 18:22] = dSwingTorque_dAngRef
            dSwingTorque_dXs[:, [6, 7, 15, 16]] = dSwingTorque_dAng
            dSwingTorque_dCon[:, num_high_con:num_high_con+num_local_con] = dSwingTorque_dConLow
            
            dfSw_dXs = np.dot(dfSw_dSwingPoint, np.dot(dSwingPoint_dPredict
                                            , dPredictSwing_dxs))
            dfSw_dXs += dfSw_dSwingAngs
            
            dfSw_dCon[:, :num_high_con] += np.dot(dfSw_dSwingPoint
                                                 , np.dot(dSwingPoint_dPredict
                                                , dPredictSwing_dCon))
            
            #dfSw_dCon[:, -1:] += np.dot(dfSw_dSwingPoint, np.dot(dSwingPoint_dPredict
            #                             , dPredictSwing_dConExp))

        return (swing_torque, dSwingTorque_dXs, dSwingTorque_dCon,
                fSw, dfSw_dXs, dfSw_dCon)
    
    def gait2dpi_u(self, xs, xsd, vs, u_close, u_stanceL, u_stanceR, u_ankle, iniL, iniR):
        
        # separate angle, anglar velocity, and anglar acceleration
        
        x = xs[:9]
        xd = xs[9:18]
        xdd = xsd[9:18]
        
        # gait 2d dynamics functions' residule, derivatives
        QQ, dQQ_dx, dQQ_dxd, dQQ_dxdd, GRF, dGRF_dx, dGRF_dxd, sticks = \
                                    autolev_rhs(x, xd, xdd, vs, self.constants_dict)
                                    

        if iniL[0] and iniR[0]:
            left_torque = u_stanceL
            right_torque = u_stanceR
            
            torque = np.zeros(9)
            torque[3:5] = left_torque
            torque[5] = u_ankle[0]
            torque[6:8] = right_torque
            torque[8] = u_ankle[1]
            
            dTorque_dConLOpen = np.zeros((9, 2))
            dTorque_dConROpen = np.zeros((9, 2))
            dTorque_dConAnkle = np.zeros((9, 2))
            dTorque_dConLOpen[3, 0] = 1
            dTorque_dConLOpen[4, 1] = 1
            dTorque_dConAnkle[5, 0] = 1
            dTorque_dConROpen[6, 0] = 1
            dTorque_dConROpen[7, 1] = 1
            dTorque_dConAnkle[8, 1] = 1
    
            f = np.zeros(self.num_states)
            
            f[:9] = xsd[:9] - xs[9:18]
            f[9:18] = (torque - QQ)/self.scaling
             
            df_dXs = np.zeros((self.num_states, self.num_states))
            df_dXsd = np.zeros((self.num_states, self.num_states))
            df_dConLOpen = np.zeros((self.num_states, 2))
            df_dConROpen = np.zeros((self.num_states, 2))
            df_dConAnkle = np.zeros((self.num_states, 2))
            df_dConClose = np.zeros((self.num_states, len(u_close)))
            
            df_dXs[:9,9:18] = -np.eye(9)
            df_dXs[9:18,:9] = (-np.reshape(dQQ_dx, (9, 9)))/self.scaling
            df_dXs[9:18,9:18] = (-np.reshape(dQQ_dxd, (9, 9)))/self.scaling
            
            df_dXs[10, 9] = 0.0
    		# force row 10 col 9 element in dfdx be zero, since it 
            # sometimes is zero, but sometimes is very small number (10^-10).
            # This caused jacobian structure unstable. Thereforce its element
            # is force to be zero here.
            
            df_dXsd[:9,:9] = np.eye(9)
            df_dXsd[9:18,9:18] = -np.reshape(dQQ_dxdd, (9, 9))/self.scaling
                        
            df_dConLOpen[9:18, :] = dTorque_dConLOpen/self.scaling
            df_dConROpen[9:18, :] = dTorque_dConROpen/self.scaling
            df_dConAnkle[9:18, :] = dTorque_dConAnkle/self.scaling
            
        elif iniL[0]:
            
            left_torque = u_stanceL
            
            (right_torque, dRightTorque_dXs, dRightTorque_dCon,
                fRight, dfRight_dXs, dfRight_dCon) =\
             self.swing_control(xs, u_close, iniR[1:3], iniR[3:5], sta_left=0, sta_right=1)
            
            torque = np.zeros(9)
            torque[3:5] = left_torque
            torque[5] = u_ankle[0]
            torque[6:8] = right_torque
            torque[8] = u_ankle[1]
            
            dTorque_dConClose = np.zeros((9, len(u_close)))
            dTorque_dConLOpen = np.zeros((9, 2))
            dTorque_dConAnkle = np.zeros((9, 2))
            
            dTorque_dConClose[6:8, :] = dRightTorque_dCon
            dTorque_dConLOpen[3, 0] = 1
            dTorque_dConLOpen[4, 1] = 1
            dTorque_dConAnkle[5, 0] = 1
            dTorque_dConAnkle[8, 1] = 1
            
            dTorque_dXs = np.zeros((9, self.num_states))
            dTorque_dXs[6:8, :] = dRightTorque_dXs
    
            f = np.zeros(self.num_states)
            
            f[:9] = xsd[:9] - xs[9:18]
            f[9:18] = (torque - QQ)/self.scaling
            f[18:22] = fRight
             
            df_dXs = np.zeros((self.num_states, self.num_states))
            df_dXsd = np.zeros((self.num_states, self.num_states))
            df_dConLOpen = np.zeros((self.num_states, 2))
            df_dConROpen = np.zeros((self.num_states, 2))
            df_dConAnkle = np.zeros((self.num_states, 2))
            df_dConClose = np.zeros((self.num_states, len(u_close)))
            
            df_dXs[:9,9:18] = -np.eye(9)
            df_dXs[9:18,:9] = (dTorque_dXs[:, :9]-np.reshape(dQQ_dx, (9, 9)))/self.scaling
            df_dXs[9:18,9:18] = (dTorque_dXs[:, 9:18]-np.reshape(dQQ_dxd, (9, 9)))/self.scaling
            df_dXs[9:18, 18:22] = dTorque_dXs[:, 18:22]/self.scaling
            df_dXs[18:22,:] = dfRight_dXs
            
            df_dXs[10, 9] = 0.0
    		# force row 10 col 9 element in dfdx be zero, since it 
            # sometimes is zero, but sometimes is very small number (10^-10).
            # This caused jacobian structure unstable. Thereforce its element
            # is force to be zero here.
            
            df_dXsd[:9,:9] = np.eye(9)
            df_dXsd[9:18,9:18] = -np.reshape(dQQ_dxdd, (9, 9))/self.scaling
                       
            df_dConLOpen[9:18, :] = dTorque_dConLOpen/self.scaling
            df_dConAnkle[9:18, :] = dTorque_dConAnkle/self.scaling
            
            df_dConClose[9:18, :] = dTorque_dConClose/self.scaling
            df_dConClose[18:22, :] = dfRight_dCon
            
        elif iniR[0]:
            
            (left_torque, dLeftTorque_dXs, dLeftTorque_dCon,
                fLeft, dfLeft_dXs, dfLeft_dCon) =\
             self.swing_control(xs, u_close, iniL[1:3], iniL[3:5], sta_left=1, sta_right=0)
            
            right_torque = u_stanceR
        
            torque = np.zeros(9)
            torque[3:5] = left_torque
            torque[5] = u_ankle[0]
            torque[6:8] = right_torque
            torque[8] = u_ankle[1]
            
            dTorque_dConClose = np.zeros((9, len(u_close)))
            dTorque_dConROpen = np.zeros((9, 2))
            dTorque_dConAnkle = np.zeros((9, 2))
            
            dTorque_dConClose[3:5, :] = dLeftTorque_dCon
            dTorque_dConAnkle[5, 0] = 1
            dTorque_dConROpen[6, 0] = 1
            dTorque_dConROpen[7, 1] = 1
            dTorque_dConAnkle[8, 1] = 1
            
            dTorque_dXs = np.zeros((9, self.num_states))
            dTorque_dXs[3:5, :] = dLeftTorque_dXs
    
            f = np.zeros(self.num_states)
            
            f[:9] = xsd[:9] - xs[9:18]
            f[9:18] = (torque - QQ)/self.scaling
            f[18:22] = fLeft
             
            df_dXs = np.zeros((self.num_states, self.num_states))
            df_dXsd = np.zeros((self.num_states, self.num_states))
            df_dConLOpen = np.zeros((self.num_states, 2))
            df_dConROpen = np.zeros((self.num_states, 2))
            df_dConAnkle = np.zeros((self.num_states, 2))
            df_dConClose = np.zeros((self.num_states, len(u_close)))
            
            df_dXs[:9,9:18] = -np.eye(9)
            df_dXs[9:18,:9] = (dTorque_dXs[:, :9]-np.reshape(dQQ_dx, (9, 9)))/self.scaling
            df_dXs[9:18,9:18] = (dTorque_dXs[:, 9:18]-np.reshape(dQQ_dxd, (9, 9)))/self.scaling
            df_dXs[9:18, 18:22] = dTorque_dXs[:, 18:22]/self.scaling
            df_dXs[18:22,:] = dfLeft_dXs
            
            df_dXs[10, 9] = 0.0
    		# force row 10 col 9 element in dfdx be zero, since it 
            # sometimes is zero, but sometimes is very small number (10^-10).
            # This caused jacobian structure unstable. Thereforce its element
            # is force to be zero here.
            
            df_dXsd[:9,:9] = np.eye(9)
            df_dXsd[9:18,9:18] = -np.reshape(dQQ_dxdd, (9, 9))/self.scaling
                       
            df_dConROpen[9:18, :] = dTorque_dConROpen/self.scaling
            df_dConAnkle[9:18, :] = dTorque_dConAnkle/self.scaling
            
            df_dConClose[9:18, :] = dTorque_dConClose/self.scaling
            df_dConClose[18:22, :] = dfLeft_dCon
        
        return f, df_dXs, df_dXsd, df_dConClose, df_dConLOpen, df_dConROpen, df_dConAnkle