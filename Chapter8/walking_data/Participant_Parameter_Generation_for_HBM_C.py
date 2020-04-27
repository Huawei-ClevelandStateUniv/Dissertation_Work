#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May  3 10:44:32 2018

This code is to calcuate joint motions from mocap data

@author: huawei
"""
import os
import numpy as np
import yaml


with open("raw_data/F0_S0/meta.yml", 'r') as stream:
    try:
        Param = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
        
        
mass = Param['subject']['mass']  # mass in kg
height = Param['subject']['height']  # height in m
knee_width = (Param['subject']['knee-width-left'] 
                + Param['subject']['knee-width-right'])/2000 # knee width in m

ankle_width = (Param['subject']['ankle-width-left'] 
                + Param['subject']['ankle-width-right'])/2000 # ankle width in m

file = 'raw_data/F0_S0/mocap'

with open('')

# Input file for inverse dynamics
# This is the current standard test for HBM
# 
bodymass 		82.5			# subjects body mass in kg
kneewidth 		0.10			# knee width in m
anklewidth		0.07			# ankle width in m
gender			M				# M or F
markerdiameter	0.015			# diameters of the markers used
filter			4.0				# low pass filter (in Hz)
kin_tolerance	0.000			# stop criterion for HBM_kinsolve 
kin_time		1e6			# max time (s) allowed for HBM_kinsolve in each frame (use 0.001 for real-time)
kin_iterations	2				# max number of iterations in HBM_kinsolve
mus_tolerance	0.000			# stop criterion for HBM_mussolve
mus_time		1e6			# max time (s) allowed for HBM_mussolve in each frame (use 0.005 for real-time)
mus_iterations  100			# max number of iterations in HBM_mussolve
exponent		2				# exponent 2 means quadratic cost function
volumeweighting 0				# 1 turns on volume weighting in cost function
markers			../../../../caren3/resources/hbm/HBMmarkers.txt
muscles			../../../../caren3/resources/hbm/HBMmuscles.txt
#frames			1 10			# do only these frames in the next file
momentarm_map					# make the file momentarm_map.csv
file 			Test1_OMC		# data file
end								