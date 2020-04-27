#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  1 16:02:38 2017

This code is to define the optimization functions for full state proportional-derivative (FPD)
gains identification of the standing balance tasks.

@author: huawei
"""
import numpy as np
from numpy import sin, cos


class Model(object):
    
    '''
    This class included the objective, gradiant, constarints, jacobian, and 
    the sparse sturcture of jacobian, which are for the ipopt optimization.

    Multiple episodes function is included.     
    '''
    
    
    def __init__(self, x_meas, accel_meas, num_repeat, num_nodes, num_states, delay_nodes, interval,
                 hidden_layer, hidden_nodes, parameters, noise, scaling = 1.0, 
                 integration_method ='backward euler'):
        
        '''
        Initialize the optimization problem.
        
        Inputs:
            x_meas:         Array (N*s*e, ), experiemntal data of the model states
                            in a sequence of X in node 0,  X in node 1, ....
            accel_meas:     Array (N*e, ), acceleration of the experiemntal perturbation
            num_eposides:   Integer, number of episodes used in the optimization
            num_nodes:      Integer, number of collocation nodes of the optimization
            num_states:     Integer, number of state of the system plant
            num_par:        Integer, number of the total optimized parameters
            interal:        Float, time interval between collocation nodes
            parameters:     Dict, personal parameters of the system plant
            Noise:          Float, amplitude of the noise in each eposide
            scaling:        Float, scaling factor for the dynamic function of the system model
            integration_method:  String, methods used to do the discrete in direct collocation

        '''
        
        self.x_meas = x_meas
        self.accel_meas = accel_meas
        self.num_repeat = num_repeat
        self.num_nodes = num_nodes
        self.interval = interval
        self.parameters = parameters
        self.scaling = scaling
        self.intergration_method = integration_method
        
        self.num_states = num_states
        self.num_cons = num_states
        self.num_conspernode = num_states
        self.conout = int(num_states/2)
        self.Noise = noise
        
        self.delay_nodes = delay_nodes
        
        self.hidden_nodes = hidden_nodes
        self.hidden_layer = hidden_layer
        
        self.num_par = ((self.num_states*(delay_nodes+1)+1)*hidden_nodes 
                       + (hidden_layer-1)*(hidden_nodes+1)*hidden_nodes 
                       + (hidden_nodes+1)*self.conout)
        
        self.itheta_a = np.linspace(0, self.num_states*self.num_nodes*self.num_repeat,
                                    self.num_nodes*self.num_repeat, endpoint=False, dtype=int)
        self.itheta_h = np.linspace(0, self.num_states*self.num_nodes*self.num_repeat,
                                    self.num_nodes*self.num_repeat, endpoint=False, dtype=int) + 1
                                 
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        f_theta_a = np.sum((x[self.itheta_a] - self.x_meas[self.itheta_a])**2)
        f_theta_h = np.sum((x[self.itheta_h] - self.x_meas[self.itheta_h])**2)
        
        obj = self.interval*(f_theta_a + f_theta_h)
        
        return  obj
    
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        
        grad = np.zeros_like(x)
        grad[self.itheta_a] = 2.0*self.interval*(x[self.itheta_a] - self.x_meas[self.itheta_a])
        grad[self.itheta_h] = 2.0*self.interval*(x[self.itheta_h] - self.x_meas[self.itheta_h])
        
        return grad

    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        N = self.num_nodes
        S = self.num_states
        R = self.num_repeat
        C = self.num_cons
        h = self.interval
        P = self.num_par
        D = self.delay_nodes
        
        a = self.accel_meas
        cons = np.zeros((C*(N - 1)*R + (R-1)*C))
        par = x[-P:]
        
        
        for q in range(R):
            for p in range(N-1):
                
                xp = x[q*N*S + (p)*S:q*N*S + (p+1)*S]
                xn = x[q*N*S + (p+1)*S:q*N*S + (p+2)*S]
                
                xdelp = np.array([])
                xdeln = np.array([])
                
                for m in range(D):
                    if m < p:
                        xdelp = np.hstack((xdelp, x[q*N*S + (p-m-1)*S : q*N*S + (p-m)*S]))
                        xdeln = np.hstack((xdeln, x[q*N*S + (p-m)*S : q*N*S + (p-m+1)*S]))
                    else:
                        xdelp = np.hstack((xdelp, np.zeros(S)))
                        xdeln = np.hstack((xdeln, np.zeros(S)))
    
                ap = a[q*N +p]
                an = a[q*N +p+1]
                
                noise = self.Noise[q*N +p,:]
                
                if self.intergration_method == 'midpoint':
                    f, dfdx, dfdxdel, dfdxdot, dfdp = self.dynamic_fun((xn + xp)/2,
                                                                       (xdeln + xdelp)/2,
                                                                       (xn - xp)/h,
                                                                       par,
                                                                       (ap + an)/2,
                                                                       noise)
                else:
                    print ('Do not have the Intergration Method code')
                    
                cons[q*(N-1)*S + S*p : q*(N-1)*S + S*(p+1)] = f
                
        for q1 in range(R-1):
            cons[R*S*(N-1)+q1*S:R*S*(N-1)+(q1+1)*S] = x[q1*N*S:q1*N*S+S] - x[(q1+1)*N*S:(q1+1)*N*S+S]
                 
        return cons
    
    def jac_index(self, k):
        #
        # manually specify the jacobian structure on one block evaluation 
        #
        
        S = self.num_states
        P = self.num_par
        D = self.delay_nodes
        
        I = min(k, D)
                       
        x = np.array([])
        y = np.array([])
        
        x = np.hstack((x, np.array([0, 0, 0, 0, 1, 1, 1, 1])))

        y = np.hstack((y, I*S + np.array([0, 2, 4, 6, 1, 3, 5, 7])))
        
        for m in range(I):
            x = np.hstack((x, np.array([2, 2, 2, 2])))
            y = np.hstack((y, np.array([m*S, m*S+1, m*S+2, m*S+3])))
            
        x = np.hstack((x, np.linspace(2, 2, 2*S+P)))

        y = np.hstack((y, I*S + np.linspace(0, 2*S+P-1, 2*S+P)))

        for m in range(I):
            x = np.hstack((x, np.array([3, 3, 3, 3])))
            y = np.hstack((y, np.array([m*S, m*S+1, m*S+2, m*S+3])))
            
        x = np.hstack((x, np.linspace(3, 3, 2*S+P)))

        y = np.hstack((y, I*S + np.linspace(0, 2*S+P-1, 2*S+P)))
        
        x = x.astype(int)
        y = y.astype(int)
        
        return x, y
        
    
    def jacobianstructure(self):
        # Sepcify the structure of Jacobian , to store the the jacobian in sparse structure.
        
        N = self.num_nodes
        S = self.num_states
        R = self.num_repeat
        C = self.num_cons
        P = self.num_par
        D = self.delay_nodes
                    
        self.Row = np.array([])
        self.Col = np.array([])
        
        for i in range(0, R):
            
            for j in range(0, N-1):
                
                I = min(j, D)
    
                self.Row = np.hstack((self.Row, i*(N-1)*S + C*j + np.array([0, 0, 0, 0, 1, 1, 1, 1])))
        
                self.Col = np.hstack((self.Col, i*N*S + S*j + np.array([0, 2, 4, 6, 1, 3, 5, 7])))
                
                for m in range(I):
                    self.Row = np.hstack((self.Row, i*(N-1)*S + C*j + np.array([2, 2, 2, 2])))
                    self.Col = np.hstack((self.Col, i*N*S + (j-I)*S + np.array([m*S, m*S+1, m*S+2, m*S+3])))
                    
                self.Row = np.hstack((self.Row, i*(N-1)*S + C*j + np.linspace(2, 2, 2*S+P)))
        
                self.Col = np.hstack((self.Col, i*N*S + j*S + np.linspace(0, 2*S-1, 2*S)))
                self.Col = np.hstack((self.Col, N*S*R + np.linspace(0, P-1, P)))
        
                for m in range(I):
                    self.Row = np.hstack((self.Row, i*(N-1)*S + C*j + np.array([3, 3, 3, 3])))
                    self.Col = np.hstack((self.Col, i*N*S + (j-I)*S + np.array([m*S, m*S+1, m*S+2, m*S+3])))
                    
                self.Row = np.hstack((self.Row, i*(N-1)*S + C*j + np.linspace(3, 3, 2*S+P)))
        
                self.Col = np.hstack((self.Col, i*N*S + j*S + np.linspace(0, 2*S-1, 2*S)))
                self.Col = np.hstack((self.Col, N*S*R + np.linspace(0, P-1, P)))
                
        self.Row = self.Row.astype(int)
        self.Col = self.Col.astype(int)
        
        for i1 in range(R-1):
            self.Row = np.hstack((self.Row, R*S*(N-1) + i1*S + np.array([0, 0, 1, 1, 2, 2, 3, 3])))
            self.Col = np.hstack((self.Col, np.array([i1*N*S, (i1+1)*N*S, i1*N*S+1, (i1+1)*N*S+1, 
                                                      i1*N*S+2, (i1+1)*N*S+2, i1*N*S+3, (i1+1)*N*S+3])))

        return (self.Row, self.Col)

    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        N = self.num_nodes
        P = self.num_par
        h = self.interval
        S = self.num_states
        R = self.num_repeat
        C = self.num_cons
        D = self.delay_nodes
        a = self.accel_meas
        
        par = x[-P:]
        
        Jac = np.array([])
        
        xl, yl = self.jac_index(D)
        
        for j in range(R):
            for k in range(N-1):
                
                xp = x[j*N*S + (k)*S : j*N*S + (k+1)*S]
                xn = x[j*N*S + (k+1)*S : j*N*S + (k+2)*S]
                
                xdelp = np.array([])
                xdeln = np.array([])
                for m in range(D):
                    if m < k:
                        xdelp = np.hstack((xdelp, x[j*N*S + (k-m-1)*S:j*N*S + (k-m)*S]))
                        xdeln = np.hstack((xdeln, x[j*N*S + (k-m)*S:j*N*S + (k-m+1)*S]))
                    else:
                        xdelp = np.hstack((xdelp, np.zeros(S)))
                        xdeln = np.hstack((xdeln, np.zeros(S)))
                
                ap = a[j*N + k]
                an = a[j*N + k+1]
                
                noise = self.Noise[j*N + k, :]
    
                if self.intergration_method == 'midpoint':
    
                    
                    f, dfdx, dfdxdel, dfdxdot, dfdp = self.dynamic_fun((xn + xp)/2,
                                                                       (xdeln + xdelp)/2,
                                                                       (xn - xp)/h,
                                                                       par,
                                                                       (ap + an)/2,
                                                                       noise)
                    
                    I = min(k, D)
                    Jac_part = np.zeros((C, (2+I)*S+P))
                    for n in range(D):
                        if n < k:
                            Jac_part[:, (I-n-1)*S:(I-n)*S] += dfdxdel[:,n*S:(n+1)*S]/2
                            Jac_part[:, (I-n)*S:(I-n+1)*S] += dfdxdel[:,n*S:(n+1)*S]/2
                                         
                    Jac_part[:, I*S:(I+1)*S] += dfdx/2 - dfdxdot/h
                    Jac_part[:, (I+1)*S:(I+2)*S] += dfdx/2 + dfdxdot/h
                    Jac_part[:, (I+2)*S:(I+2)*S+P] += dfdp
                        
                    if k < D:
                        xi, yi = self.jac_index(k)
                        
                        RA = Jac_part[xi, yi]
                    else:
                        RA = Jac_part[xl, yl]
                            
                    Jac = np.hstack((Jac, RA))
                    
                else:
                    print ('Do not have the Intergration Method code')
                    
        for j1 in range(R-1):
            Jac = np.hstack((Jac, np.array([1, -1, 1, -1, 1, -1, 1, -1]) ))
           
        return Jac
    
        
    def dynamic_fun(self, x, xdel, xdot, p, a, noise):
        #
        # The dynamic equation of the simulation feedback model, here is the constraints
        # of the optimization problem.
        #
        # The system model is a double-link pendulum with feedback controls
        #
        
        l_L = self.parameters[0]
        I_T = self.parameters[6]/self.scaling
        g = 9.81
        d_T = self.parameters[2]
        d_L = self.parameters[1]
        I_L = self.parameters[5]/self.scaling
        m_L = self.parameters[3]/self.scaling
        m_T = self.parameters[4]/self.scaling
                             
        theta_a = x[0]
        theta_h = x[1]
        omega_a = x[2]
        omega_h = x[3]
        
        theta_a_dot = xdot[0]
        theta_h_dot = xdot[1]
        omega_a_dot = xdot[2]
        omega_h_dot = xdot[3]
        
        S = self.num_states
        PN = self.num_par
        
        f = np.zeros((S))
        dfdx = np.zeros((S, S))
        dfdxdel = np.zeros((S, len(xdel)))
        dfdxdot = np.zeros((S, S))
        dfdp = np.zeros((S, PN))
        
        Tor, dTordp, dTordx, dTordxdel = self.neurnet_con(x, xdel, p)

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
        
        return f, dfdx, dfdxdel, dfdxdot, dfdp
    
    def Afun(self, a):
        #
        # The smoothed activate function of the neural network 
        #
        
        k = 50
        f = 0.7/k*np.log(1+np.exp(-k*a)) + a
        
        return f
    
    def Cdfda(self, a):
        #
        # calcuate df_da for the derivatives of the neural network controller
        #
        k = 50
        Len = a.size
        dfda = np.zeros((Len+1, Len))
        
        for i in range(Len):
            dfda[i, i] = 0.7/(1+np.exp(-k*a[i])) + 0.3
                
        return dfda
    
    def Cdadw (self, f):
        #
        # calcuate df_dw for the derivatives of the neural network controller
        #
        Len = f.size
        
        HN = self.hidden_nodes
        
        dadw = np.zeros((HN, HN*Len))
        
        for i in range(HN):
            dadw[i, i*Len:(i+1)*Len] = f
                 
        return dadw
    
    def neurnet_con(self, x, xdel, p):
        #
        # function of neural network control, calcuate controller outputs as 
        # well as derivatives
        #        
        PN = self.num_par
        S = self.num_states
        CO = self.conout
        D = self.delay_nodes
        cons = 1
        HL = self.hidden_layer
        HN = self.hidden_nodes
        
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
            
            fv[:-1,k] = self.Afun(av[:, k])
            fv[-1,k] = cons
            av[:,k+1] = np.dot(weight_hl[k,:,:], fv[:,k])
                  
        fv[:-1,HL-1] = self.Afun(av[:, HL-1])       
        fv[-1,HL-1] = cons
          
        Tor = np.dot(weight_e, fv[:,HL-1])
        
        dtordp[:, -CO*(HN+1):] = np.array([np.hstack([fv[:, HL-1], np.zeros((HN+1))]),
                                           np.hstack([np.zeros((HN+1)), fv[:, HL-1]])])
                              
        for l in range(HL):
            if l == 0:
                dtorda[HL-1-l,:,:] = np.dot(weight_e, self.Cdfda(av[:,HL-1-l]))            
            else:
                dtorda[HL-1-l,:,:] = np.dot(np.dot(dtorda[HL-l,:,:], weight_hl[HL-1-l,:,:]), self.Cdfda(av[:,HL-1-l]))
                
            if l < HL-1: 
                dtordp[:, -(HN*(HN+1)*(l+1)+CO*(HN+1)):-(HN*(HN+1)*l+CO*(HN+1))] = np.dot(dtorda[HL-1-l,:,:], self.Cdadw(fv[:,HL-2-l]))
            else:
                dtordp[:, :(S*(D+1)+1)*HN] = np.dot(dtorda[0, :, :], self.Cdadw(Input))
                
        dtordinp = np.dot(dtorda[0, :, :], weight_b)
        
        dtordx = dtordinp[:, :len(x)]
        dtordxdel = dtordinp[:, len(x):len(x)+len(xdel)]
    
        return Tor, dtordp, dtordx, dtordxdel
    
    

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