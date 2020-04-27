#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 30 13:34:36 2017

@author: huawei
"""
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

def animate_pendulum(t, sticks, belt, fps_def, filename=None):
    """Animates the 7-pendulum walking model and optionally saves it to file.

    Parameters
    ----------
    t : ndarray, shape(m)
        Time array.
    states: ndarray, shape(m,p)
        State time history.
    filename: string or None, optional
        If true a movie file will be saved of the animation. This may take some time.

    Returns
    -------
    fig : matplotlib.Figure
        The figure.
    anim : matplotlib.FuncAnimation
        The animation.

    """

    # first set up the figure, the axis, and the plot elements we want to animate
    fig = plt.figure(figsize=(12, 12))
    
    # create the axes
    ax = plt.axes(xlim=(-1.5, 1.5), ylim=(-0.1, 1.8), aspect='equal')
    
    # display the current time
    time_text = ax.text(0.04, 0.9, '', transform=ax.transAxes)
    
    # create a ground with black dot
    ms = 8
    line_dot0, = ax.plot([], [], 'k.', markersize=ms)
    line_dot1, = ax.plot([], [], 'k-', linewidth=1)
    
    # blank line for the pendulum
    line_link_r, = ax.plot([], [], 'r', lw=2)
    line_link_l, = ax.plot([], [], 'b', lw=2)

    # initialization function: plot the background of each frame
    def init():
        time_text.set_text('')
        line_dot0.set_data([], [])
        line_dot1.set_data([], [])
        
        line_link_r.set_data([], [])
        line_link_l.set_data([], [])
        return (time_text, line_link_r, line_link_l, line_dot0, line_dot1)

    # animation function: update the objects
    def animate(i):
        time_text.set_text('time = {:2.2f}'.format(t[i]))
        

        x_d0 = np.linspace(-2.5, abs(belt[-1])+5.0, 100) + belt[i]
        y_d0 = np.zeros(100) + 0.02
        x_d1 = np.array([-2.5, 2.5])
        y_d1 = np.array([-0.0, -0.0])
        

                       
        x_r = sticks[:, 2*np.array([0,1,2,3,4,5,3])]
        y_r = sticks[:, 2*np.array([0,1,2,3,4,5,3])+1]
        x_l = sticks[:, 2*np.array([1,6,7,8,9,7])]
        y_l = sticks[:, 2*np.array([1,6,7,8,9,7])+1]

        line_link_l.set_data(x_l[i, :], y_l[i, :])
        line_link_r.set_data(x_r[i, :], y_r[i, :])
        
        line_dot0.set_data(x_d0, y_d0)
        line_dot1.set_data(x_d1, y_d1)
        
        return (time_text, line_link_l, line_link_r, line_dot0, line_dot1)

    # call the animator function
    anim = animation.FuncAnimation(fig, animate, frames=len(t), init_func=init,
            interval=t[-1] / len(t), blit=True, repeat=False)
    
    # save the animation if a filename is given
    if filename is not None:
        anim.save(filename, fps=fps_def, codec='libx264', bitrate=-1)
