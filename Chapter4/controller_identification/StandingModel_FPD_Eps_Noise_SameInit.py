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
    
    def __init__(self, x_meas, accel_meas, num_repeat, num_nodes, num_states, num_par,
                 interval, parameters, noise, scaling =1, integration_method='backward euler'):
        
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
        self.num_par = num_par
        self.num_conspernode = num_states
        
        self.Noise = noise
        
        self.Row_part, self.Col_part = self.Jac_Index()
        
        self.itheta_a = np.array([])
        self.itheta_h = np.array([])

        
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
        
        return  self.interval*(f_theta_a + f_theta_h)
    
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        
        grad = np.zeros_like(x)
        
#        grad[:self.num_states*self.num_nodes] = 2.0*self.interval*(x[:self.num_states*self.num_nodes] - self.x_meas)
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
        h = self.interval
        a = self.accel_meas
        
        cons = np.zeros((R*S*(N-1)+(R-1)*S))
        par = x[-self.num_par:]
        
        for q in range(R):
            for p in range(N-1):
                
                xp = x[q*N*S + (p)*S:q*N*S + (p+1)*S]
                xn = x[q*N*S + (p+1)*S:q*N*S + (p+2)*S]
    
                ap = a[q*N + p]
                an = a[q*N + p+1]
                
                noise = self.Noise[q*N + p, :]
                           
                if self.intergration_method == 'midpoint':
                    f, dfdx, dfdxdot, dfdp = self.dynamic_fun((xp + xn)/2, (xn - xp)/h, par, (ap + an)/2, noise)
                else:
                    print ('Do not have the Intergration Method code')
                    
                cons[q*(N-1)*S +S*p: q*(N-1)*S +S*(p+1)] = f
                
        for q1 in range(R-1):
            cons[R*S*(N-1)+q1*S:R*S*(N-1)+(q1+1)*S] = x[q1*N*S:q1*N*S+S] - x[(q1+1)*N*S:(q1+1)*N*S+S]
        
                 
        return cons
    
    def jacobianstructure(self):
        #
        # Sepcify the structure of Jacobian, to store the the jacobian in sparse structure.
        #
        
        N = self.num_nodes
        S = self.num_states
        C = self.num_cons
        R = self.num_repeat
        
        self.Row = np.array([])
        self.Col = np.array([])
        
        for i in range(0, R):
            for j in range(0, N-1):
                              
                x = np.array([j*C, j*C, j*C, j*C, j*C+1, j*C+1, j*C+1, j*C+1,
                          
                          j*C+2, j*C+2, j*C+2, j*C+2, j*C+2, j*C+2, j*C+2, j*C+2,
                          j*C+2, j*C+2, j*C+2, j*C+2, j*C+2, j*C+2,
                          
                          j*C+3, j*C+3, j*C+3, j*C+3, j*C+3, j*C+3, j*C+3, j*C+3,
                          j*C+3, j*C+3, j*C+3, j*C+3, j*C+3, j*C+3
                          ]) + i*(N-1)*S
    
                y = np.array([i*N*S+j*S, i*N*S+j*S+2, i*N*S+j*S+4, i*N*S+j*S+6, 
                              i*N*S+j*S+1, i*N*S+j*S+3, i*N*S+j*S+5, i*N*S+j*S+7,
                          
                          i*N*S+j*S, i*N*S+j*S+1, i*N*S+j*S+2, i*N*S+j*S+3, i*N*S+j*S+4, 
                          i*N*S+j*S+5, i*N*S+j*S+6, i*N*S+j*S+7, R*N*S, R*N*S+1, R*N*S+2,
                          R*N*S+3, R*N*S+8, R*N*S+9,
                          
                          i*N*S+j*S, i*N*S+j*S+1, i*N*S+j*S+2, i*N*S+j*S+3, i*N*S+j*S+4,
                          i*N*S+j*S+5, i*N*S+j*S+6, i*N*S+j*S+7, R*N*S+4, R*N*S+5, R*N*S+6,
                          R*N*S+7, R*N*S+8, R*N*S+9
                          ])
        
                x.astype(int)
                y.astype(int)
                
                self.Row = np.hstack((self.Row, x))
                self.Col = np.hstack((self.Col, y))
                
        for i1 in range(R-1):
            self.Row = np.hstack((self.Row, R*S*(N-1) + i1*S + np.array([0, 0, 1, 1, 2, 2, 3, 3])))
            self.Col = np.hstack((self.Col, np.array([i1*N*S, (i1+1)*N*S, i1*N*S+1, (i1+1)*N*S+1, 
                                                      i1*N*S+2, (i1+1)*N*S+2, i1*N*S+3, (i1+1)*N*S+3])))
        
        return (self.Row, self.Col)
    
    
    
    def Jac_Index(self, ):
        #
        # manually specify the jacobian structure on one block evaluation 
        #
        
        self.Row_part = np.array([0, 0, 0, 0, 1, 1, 1, 1,
                                  
                                  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                                  
                                  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
                                  ])
    
        self.Col_part = np.array([0, 2, 4, 6, 1, 3, 5, 7,
                                                                    
                                  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17,
                                  
                                  0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 17
                                  ])
    
        return self.Row_part, self.Col_part

    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        N = self.num_nodes
        P = self.num_par
        h = self.interval
        S = self.num_states
        C = self.num_cons
        R = self.num_repeat
        
        a = self.accel_meas
        
        par = x[-self.num_par:]
                
        Jac_part = np.zeros((C, 2*S+P))
        
        Jac = np.array([])
        
        for j in range(R):
            for k in range(N-1):
                
                xp = x[j*N*S + (k)*S:j*N*S + (k+1)*S]
                xn = x[j*N*S + (k+1)*S:j*N*S + (k+2)*S]
                
                ap = a[j*N + k]
                an = a[j*N + k+1]
                
                noise = self.Noise[j*N + k, :]
                
                if self.intergration_method == 'midpoint':
                    f, dfdx, dfdxdot, dfdp = self.dynamic_fun((xp + xn)/2, (xn - xp)/h, par, (ap + an)/2, noise)
                    
                    Jac_part[:, :S] = dfdx/2 -dfdxdot/h
                    Jac_part[:, S:2*S] = dfdx/2 + dfdxdot/h
                            
                    Jac_part[:, 2*S:2*S+P] = dfdp
                             
                    RA = Jac_part[self.Row_part, self.Col_part]
                    Jac = np.hstack((Jac, RA))
    
                else:
                    print ('Do not have the Intergration Method code')
                    
        for j1 in range(R-1):
            Jac = np.hstack((Jac, np.array([1, -1, 1, -1, 1, -1, 1, -1]) ))
           
        return Jac
        
    def dynamic_fun(self, x, xdot, p, a, noise):
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
  
        k00 = p[0]
        k01 = p[1]
        k02 = p[2]
        k03 = p[3]
        
        k10 = p[4]
        k11 = p[5]
        k12 = p[6]
        k13 = p[7]
        
        ref_a = p[8]
        ref_h = p[9]
        
        f = np.zeros((self.num_conspernode))
        dfdx = np.zeros((self.num_conspernode, self.num_states))
        dfdxdot = np.zeros((self.num_conspernode, self.num_states))
        dfdp = np.zeros((self.num_conspernode, self.num_par))
        

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
                - omega_h_dot*(I_T + d_T*m_T*(d_T + l_L*cos(theta_h)))
                - (k00*(theta_a-ref_a) + k01*(theta_h-ref_h) + k02*omega_a + k03*omega_h) - noise[0])
            
        dfdx[2,0] = (-a*d_L*m_L*sin(theta_a) - a*d_T*m_T*sin(theta_a + theta_h)
                        - a*l_L*m_T*sin(theta_a) + d_L*g*m_L*cos(theta_a) 
                        + d_T*g*m_T*cos(theta_a + theta_h) + g*l_L*m_T*cos(theta_a) - k00)
        
        dfdx[2,1] = (-a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h)
                        - d_T*l_L*m_T*omega_a**2*cos(theta_h) + 2*d_T*l_L*m_T*omega_a_dot*sin(theta_h) 
                        + d_T*l_L*m_T*omega_h_dot*sin(theta_h) + d_T*l_L*m_T*(omega_a + omega_h)**2*cos(theta_h) - k01)

        dfdx[2,2] = -2*d_T*l_L*m_T*omega_a*sin(theta_h) + d_T*l_L*m_T*(2*omega_a + 2*omega_h)*sin(theta_h) - k02
            
        dfdx[2,3] = d_T*l_L*m_T*(2*omega_a + 2*omega_h)*sin(theta_h) - k03
        
        
        dfdxdot[2,2] = -I_L - I_T - d_L**2*m_L - m_T*(d_T**2 + 2*d_T*l_L*cos(theta_h) + l_L**2)
        dfdxdot[2,3] = -I_T - d_T*m_T*(d_T + l_L*cos(theta_h))
        
        
        dfdp[2,0] = -(theta_a-ref_a)
        dfdp[2,1] = -(theta_h-ref_h)
        dfdp[2,2] = -omega_a
        dfdp[2,3] = -omega_h
        
        dfdp[2,8] = k00
        dfdp[2,9] = k01
        
        
        f[3] =  (a*d_T*m_T*cos(theta_a + theta_h) + d_T*g*m_T*sin(theta_a + theta_h) 
                - d_T*l_L*m_T*omega_a**2*sin(theta_h) - (I_T + d_T**2*m_T)*omega_h_dot 
                - (I_T + d_T*m_T*(d_T + l_L*cos(theta_h)))*omega_a_dot 
                - (k10*(theta_a-ref_a) + k11*(theta_h-ref_h) + k12*omega_a + k13*omega_h) - noise[1])

            
        dfdx[3,0] = - a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) - k10
            
        dfdx[3,1] = (-a*d_T*m_T*sin(theta_a + theta_h) + d_T*g*m_T*cos(theta_a + theta_h) 
                    - d_T*l_L*m_T*omega_a**2*cos(theta_h) + d_T*l_L*m_T*omega_a_dot*sin(theta_h) - k11)
                                         
        dfdx[3,2] = -2*d_T*l_L*m_T*omega_a*sin(theta_h) - k12

        dfdx[3,3] = -k13
        
        
        dfdxdot[3,2] = -I_T - d_T*m_T*(d_T + l_L*cos(theta_h))
        dfdxdot[3,3] = -I_T - d_T**2*m_T
        
        dfdp[3,4] = -(theta_a-ref_a)
        dfdp[3,5] = -(theta_h-ref_h)
        dfdp[3,6] = -omega_a
        dfdp[3,7] = -omega_h
        
        dfdp[3,8] = k10
        dfdp[3,9] = k11

#        
        return f, dfdx, dfdxdot, dfdp

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
        print ('Objective value at iteration #%d is - %g' % (iter_count, obj_value))