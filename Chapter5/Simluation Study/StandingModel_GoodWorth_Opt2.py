#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  1 16:02:38 2017

This is the optimization model of a single pendulum, which was controlled by
PD controller with Delay.

@author: huawei
"""
import numpy as np
#import sympy as sy
from numpy import sin, cos
from scipy.sparse import find

class Model(object):
    def __init__(self, x_meas, accel_meas, num_nodes, num_states, num_par, max_delay_time,
                interval, parameters, scaling = 1.0, integration_method ='midpoint'):
        
        self.x_meas = x_meas
        self.accel_meas = accel_meas
        self.num_nodes = num_nodes
        self.interval = interval
        self.parameters = parameters
        self.scaling = scaling
        self.intergration_method = integration_method
        
        self.num_states = num_states
        self.num_cons = num_states
        self.num_conspernode = num_states
        self.conout = int(num_states/2)
        
        self.max_dtime = max_delay_time
        
        self.num_delay = int(max_delay_time/interval)
        
        self.num_par = num_par
        
        self.itheta_a = np.linspace(0, self.num_states*self.num_nodes, self.num_nodes, endpoint=False, dtype=int)
        self.itheta_h = self.itheta_a + 1
                                 
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        
        f_theta_a = np.sum((x[self.itheta_a] - self.x_meas[self.itheta_a])**2)
        f_theta_h = np.sum((x[self.itheta_h] - self.x_meas[self.itheta_h])**2)
        
        obj = (f_theta_a + f_theta_h)/self.num_nodes*1e3
        
        return  obj
    
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        
        grad = np.zeros_like(x)
#        grad[:self.num_states*self.num_nodes] = 2.0*self.interval*(x[:self.num_states*self.num_nodes] - self.x_meas)
        grad[self.itheta_a] = 2.0*(x[self.itheta_a] - self.x_meas[self.itheta_a])/self.num_nodes*1e3
        grad[self.itheta_h] = 2.0*(x[self.itheta_h] - self.x_meas[self.itheta_h])/self.num_nodes*1e3
        
        return grad

    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        N = self.num_nodes
        S = self.num_states
        C = self.num_cons
        h = self.interval
        P = self.num_par
        D = self.num_delay
        a = self.accel_meas
        cons = np.zeros((C*(N - 1)))
        par = x[-P:]
        
        # Make sure that all delay are not integer respect to delay nodes.
        # There are 4 time delay components in total
        for dn in range(2):
#            if abs(par[2+S*2+dn]/self.interval - int(par[2+S*2+dn]/self.interval)) < 1e-6:
#                par[2+S*2+dn] += -1e-8
                
            if par[2+S*2+dn] >= self.max_dtime:
                par[2+S*2+dn] = self.max_dtime - 1e-8
        
        for p in range(N-1):
            
            xp = x[p*S:(p+1)*S]
            xn = x[(p+1)*S:(p+2)*S]
            
            for m in range(D):
                if m < p:
                    xp = np.hstack((x[(p-m-1)*S:(p-m)*S], xp))
                    xn = np.hstack((x[(p-m)*S:(p-m+1)*S], xn))
                else:
                    xp = np.hstack((x[:S], xp))
                    xn = np.hstack((x[S:2*S], xn))

            ap = a[p]
            an = a[p+1]
            
            if self.intergration_method == 'midpoint':
                f, _, _, _ = self.dynamic_fun((xn + xp)/2,
                                              (xn[-S:] - xp[-S:])/h, par,
                                              (ap + an)/2)
            else:
                print ('Do not have the Intergration Method code')
                
            cons[S*p: S*(p+1)] = f
                 
        return cons
        
    
    def jacobianstructure(self):
        # Sepcify the structure of Jacobian, in case the size of Jacobian is too large. 
        
        N = self.num_nodes
        P = self.num_par
        h = self.interval
        S = self.num_states
        C = self.num_cons
        D = self.num_delay
        a = self.accel_meas
        
        half_state = int(S/2)
        
        x_meas = 0.1*np.random.random(N*S+P)

        par = x_meas[-P:]
        
        par[0] = 1500*np.random.random(1)
        par[1] = 1500*np.random.random(1)
        
        par[2:2+half_state] = 1500*np.random.random(half_state)
        par[2+half_state:2+S] = 300*np.random.random(half_state)
        
        par[2+S:2+S+half_state] = 1500*np.random.random(half_state)
        par[2+S+half_state:2+2*S] = 300*np.random.random(half_state)
        
        par[2+2*S:2+2*S+2] = (self.max_dtime - 1e-8)*np.random.random(2) + 1e-8
        par[2+2*S+2:2+2*S+4] = 0.1 - 0.2*np.random.random(2)
        
        for dn in range(2):
#            if abs(par[2+2*S+dn]/self.interval - int(par[2+2*S+dn]/self.interval)) < 1e-6:
#                par[2+2*S+dn] += -1e-8
#                
            if par[2+S*2+dn] >= self.max_dtime:
                par[2+S*2+dn] = self.max_dtime - 1e-8
        
        self.Row = np.array([])
        self.Col = np.array([])

        for k in range(N-1):
            
            
            Jac_part = np.zeros((C, int(S*2 + S*min(k, D))))
            
            xp = x_meas[k*S:(k + 1)*S]
            xn = x_meas[(k + 1)*S:(k + 2)*S]
            
            # Make Sure that the delay nodes exist, otherwise fill delay states with the first node
            for m in range(D):
                if m < k:
                    xp = np.hstack((x_meas[(k - m - 1)*S:(k - m)*S], xp))
                    xn = np.hstack((x_meas[(k-m)*S:(k-m+1)*S], xn))
                else:
                    xp = np.hstack((x_meas[:S], xp))
                    xn = np.hstack((x_meas[S:2*S], xn))
            
            ap = a[k]
            an = a[k+1]

            if self.intergration_method == 'midpoint':
                
                _, dfdx, dfdxdot, dfdp = self.dynamic_fun((xn + xp)/2,
                                                          (xn[-S:] - xp[-S:])/h, par,
                                                          (ap + an)/2)
                
                Jac_part[:, -2*S:-S] = - dfdxdot/h
                Jac_part[:, -S:] =  + dfdxdot/h
                
                Jac_part[:int(S/2), -2*S:-S] += dfdx[:int(S/2), -S:]/2
                Jac_part[:int(S/2), -S:] += dfdx[:int(S/2), -S:]/2
                
                for n in range(D):
                     if n < k:
                        if n == 0:
                            Jac_part[int(S/2):, -2*S:-S] += dfdx[int(S/2):, -S:]/2
                            Jac_part[int(S/2):, -S:] += dfdx[int(S/2):, -S:]/2
                        else:
                            Jac_part[int(S/2):, -(2+n)*S:-(1+n)*S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                            Jac_part[int(S/2):, -(1+n)*S:-n*S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                     else:
                         if n == 0:
                             Jac_part[int(S/2):, S:2*S] += dfdx[int(S/2):, -S:]/2
                             Jac_part[int(S/2):, :S] += dfdx[int(S/2):, -S:]/2
                         else:
                            Jac_part[int(S/2):, S:2*S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                            Jac_part[int(S/2):, :S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                
                for o in range(C):
                    if o < int(S/2):
                        row_x, col_x, RA = find(Jac_part[o, :])
                        
                        row_x += k*C + o
                        col_x += k*S - min(k, D)*S
                        
                        self.Row = np.hstack((self.Row, row_x))
                        self.Col = np.hstack((self.Col, col_x))
                        
                    else:
                        row_x = np.linspace(o, o, (min(k, D)+2)*S) + k*C
                        col_x = np.linspace(0, (min(k, D)+2)*S, (min(k, D)+2)*S,
                                            endpoint=False) + k*S - min(k, D)*S
                                            
                        row_p = np.linspace(o, o, len(par)) + k*C
                        col_p = np.linspace(N*S, N*S+len(par), len(par), endpoint=False)

                        self.Row = np.hstack((self.Row, row_x, row_p))
                        self.Col = np.hstack((self.Col, col_x, col_p))
                
            else:
                print ('Do not have the Intergration Method code')
                
        self.Row = self.Row.astype(int)
        self.Col = self.Col.astype(int)

        return (self.Row, self.Col)

    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        N = self.num_nodes
        P = self.num_par
        h = self.interval
        S = self.num_states
        C = self.num_cons
        D = self.num_delay
        a = self.accel_meas
        
        par = x[-P:]
        for dn in range(2):
#            if abs(par[2+S*2+dn]/self.interval - int(par[2+S*2+dn]/self.interval)) < 1e-6:
#                par[2+S*2+dn] += -1e-8
                
            if par[2+S*2+dn] >= self.max_dtime:
                par[2+S*2+dn] = self.max_dtime - 1e-8
                
        
        Jac = np.array([])

        for k in range(N-1):
            
            Jac_part = np.zeros((C, int(S*2 + S*min(k, D))))
            
            xp = x[k*S:(k+1)*S]
            xn = x[(k+1)*S:(k+2)*S]
            
            # Make Sure that the delay nodes exist, otherwise fill delay states with first node
            for m in range(D):
                if m < k:
                    xp = np.hstack((x[(k - m - 1)*S:(k - m)*S], xp))
                    xn = np.hstack((x[(k - m)*S:(k - m + 1)*S], xn))
                else:
                    xp = np.hstack((x[:S], xp))
                    xn = np.hstack((x[S:2*S], xn))
            
            ap = a[k]
            an = a[k+1]

            if self.intergration_method == 'midpoint':

                _, dfdx, dfdxdot, dfdp = self.dynamic_fun((xn + xp)/2,
                                                          (xn[-S:] - xp[-S:])/h,
                                                          par, (ap + an)/2)
                
                Jac_part[:, -2*S:-S] = - dfdxdot/h
                Jac_part[:, -S:] = + dfdxdot/h
                
                Jac_part[:int(S/2), -2*S:-S] += dfdx[:int(S/2), -S:]/2
                Jac_part[:int(S/2), -S:] += dfdx[:int(S/2), -S:]/2
                
                for n in range(D+1):
                     if n < k:
                         if n == 0:
                            Jac_part[int(S/2):, -2*S:-S] += dfdx[int(S/2):, -S:]/2
                            Jac_part[int(S/2):, -S:] += dfdx[int(S/2):, -S:]/2
                         else:
                            Jac_part[int(S/2):, -(2+n)*S:-(1+n)*S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                            Jac_part[int(S/2):, -(1+n)*S:-n*S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                     else:
                         if n == 0:
                            Jac_part[int(S/2):, S:2*S] += dfdx[int(S/2):, -S:]/2
                            Jac_part[int(S/2):, :S] += dfdx[int(S/2):, -S:]/2
                         else:
                            Jac_part[int(S/2):, S:2*S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                            Jac_part[int(S/2):, :S] += dfdx[int(S/2):, -(1+n)*S:-n*S]/2
                        
                for o in range(C):
                    if o < int(S/2):
                        _, _, RA = find(Jac_part[o, :])
                        
                        Jac = np.hstack((Jac, RA))
                        
                    else:
                        
                        RA = Jac_part[o, :]
                        RP = dfdp[o-int(S/2), :]
                        
                        Jac = np.hstack((Jac, RA, RP))
                
            else:
                print ('Do not have the Intergration Method code')
           
        return Jac
    
        
    def dynamic_fun(self, x, xdot, p, a):

        l_L = self.parameters[0]
        I_T = self.parameters[6]/self.scaling
        g = 9.81
        d_T = self.parameters[2]
        d_L = self.parameters[1]
        I_L = self.parameters[5]/self.scaling
        m_L = self.parameters[3]/self.scaling
        m_T = self.parameters[4]/self.scaling
                             
        theta_a = x[-4]
        theta_h = x[-3]
        omega_a = x[-2]
        omega_h = x[-1]
        
        theta_a_dot = xdot[0]
        theta_h_dot = xdot[1]
        omega_a_dot = xdot[2]
        omega_h_dot = xdot[3]
        
        S = self.num_states
        PN = self.num_par
        
        f = np.zeros((S))
        dfdx = np.zeros((S, len(x)))
        dfdxdot = np.zeros((S, S))
        dfdp = np.zeros((S, PN))
        
        tor, dtor_dx, dtor_dp = self.FPD_delay(x, p)

        f[0] = omega_a - theta_a_dot
    
        dfdx[0, -2] = 1
        dfdxdot[0, 0] = -1
        
        f[1] = omega_h - theta_h_dot
        
        dfdx[1, -1] = 1
        dfdxdot[1, 1] = -1
        
        f[2] = (a*d_L*m_L*cos(theta_a) + a*d_T*m_T*cos(theta_a + theta_h) 
                + a*l_L*m_T*cos(theta_a) + d_L*g*m_L*sin(theta_a) + d_T*g*m_T*sin(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*sin(theta_h) 
                + g*l_L*m_T*sin(theta_a) - omega_a_dot*(I_L + I_T + d_L**2*m_L + m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)) 
                - omega_h_dot*(I_T + d_T*m_T*(d_T + l_L*cos(theta_h))))
  
        
        dfdx[2, -4] = (-a*d_L*m_L*sin(theta_a) - a*d_T*m_T*sin(theta_a + theta_h)
                        - a*l_L*m_T*sin(theta_a) + d_L*g*m_L*cos(theta_a) 
                        + d_T*g*m_T*cos(theta_a + theta_h) + g*l_L*m_T*cos(theta_a))
                                       
        dfdx[2, -3] = (-a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) 
                    - d_T*l_L*m_T*omega_a**2*cos(theta_h) + 2*d_T*l_L*m_T*omega_a_dot*sin(theta_h) 
                    + d_T*l_L*m_T*omega_h_dot*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*cos(theta_h))
                                                 
        dfdx[2, -2] = -2*d_T*l_L*m_T*omega_a*sin(theta_h) + d_T*l_L*m_T*(2*omega_a + 2*omega_h)*sin(theta_h)
        
        dfdx[2, -1] = d_T*l_L*m_T*(2*omega_a + 2*omega_h)*sin(theta_h)
        
        dfdxdot[2, 2] = -I_L - I_T - d_L**2*m_L - m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)
        dfdxdot[2, 3] = -I_T - d_T*m_T*(d_T + l_L*cos(theta_h))
        
        
        f[3] =  (a*d_T*m_T*cos(theta_a + theta_h) + d_T*g*m_T*sin(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*sin(theta_h) - (I_T + d_T**2*m_T)*omega_h_dot 
                - (I_T + d_T*m_T*(d_T + l_L*cos(theta_h)))*omega_a_dot)
        
        
        dfdx[3, -4] = -a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h)
        
        dfdx[3, -3] = (-a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) 
                    - d_T*l_L*m_T*omega_a**2*cos(theta_h) + d_T*l_L*m_T*omega_a_dot*sin(theta_h))
                                                                               
        dfdx[3, -2] = -2*d_T*l_L*m_T*omega_a*sin(theta_h)
            
        dfdxdot[3, 2] = -I_T - d_T*m_T*(d_T + l_L*cos(theta_h))
        dfdxdot[3, 3] = -I_T - d_T**2*m_T
        
        f[int(S/2):] = f[int(S/2):] + tor
        
        dfdx[int(S/2):, :] = dfdx[int(S/2):, :] + dtor_dx
        dfdp = dtor_dp
        
        return f, dfdx, dfdxdot, dfdp

    def FPD_delay(self, x, par):
               
        S = self.num_states
        
        tor = np.zeros(int(S/2))
        dtor_dx = np.zeros((int(S/2), len(x)))
        dtor_dp = np.zeros((int(S/2), len(par)))
        
        ref_x = np.array([par[-2], par[-1], 0, 0])
        
        for d in range(int(S/2)):
            
            K_passive = par[:2]
            K_active = par[2+d*S:2+(d+1)*S]
            
            dn1 = par[2+S*2+d]/self.interval
            #dn2 = par[2+S*2+2*d+1]/self.interval
            
            xv = ((-dn1 + 1 + int(dn1))*x[[-int(dn1+1)*S, -int(dn1+1)*S + 1,
                                          -int(dn1+1)*S + 2, -int(dn1+1)*S + 3]]
                   + (dn1 - int(dn1))*x[[-int(dn1+2)*S, -int(dn1+2)*S + 1, 
                                         -int(dn1+2)*S + 2, -int(dn1+2)*S + 3]])
                
            tor[d] = (K_passive[d]*(ref_x[d] - x[-S+d]) 
                    + np.dot(K_active, ref_x - xv))
            
            dtor_dx[d, -S+d] += -K_passive[d]
            
            dtor_dx[d, [-int(dn1+1)*S, -int(dn1+1)*S + 1,
                        -int(dn1+1)*S + 2, -int(dn1+1)*S + 3]] += -(-dn1 + 1 + int(dn1))*K_active
            dtor_dx[d, [-int(dn1+2)*S, -int(dn1+2)*S + 1, 
                        -int(dn1+2)*S + 2, -int(dn1+2)*S + 3]] += -(dn1 - int(dn1))*K_active
            
            dtor_dp[d, d] += ref_x[d] - x[-S+d]
            dtor_dp[d, 2+d*S:2+(d+1)*S] += ref_x - xv
            
            if dn1 == int(dn1):
                dtor_dp[d, 2+2*S+d] = 0
            else:
                dtor_dp[d, 2+2*S+d] += -np.dot(K_active, x[[-int(dn1+2)*S, -int(dn1+2)*S + 1, 
                                         -int(dn1+2)*S + 2, -int(dn1+2)*S + 3]] 
                                        - x[[-int(dn1+1)*S, -int(dn1+1)*S + 1,
                                          -int(dn1+1)*S + 2, -int(dn1+1)*S + 3]])/self.interval
                
                
            dtor_dp[d, -2+d] += K_passive[d]
            dtor_dp[d, -2] += K_active[0]
            dtor_dp[d, -1] += K_active[1]
        
        return tor, dtor_dx, dtor_dp
        

    def intermediate(
            self,
            alg_mod,
            iter_count,
            obj_value,
            inf_pr,
            inf_du,
            mu,
            d_norm,
            regularization_size,
            alpha_du,
            alpha_pr,
            ls_trials
            ):

        #
        # Example for the use of the intermediate callback.
        #
        print ("Objective value at iteration #%d is - %g" % (iter_count, obj_value))