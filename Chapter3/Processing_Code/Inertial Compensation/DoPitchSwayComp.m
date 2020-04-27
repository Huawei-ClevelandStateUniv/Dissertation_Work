%======================================================================
%SCRIPT inertia_compensation_test.m
%   1) Loads data files containing random treadmill movements
%   2) Filters marker, force plate, and acceleration signals
%   3) Adds a time delay to the force and acceleration signals
%   4) Passes the acceleration and the pitch moment into the
%      compensation.m function to reduce the inertial errors
%   5) Performs a coordinate transformation using the Soderquist method
%      in the transformation.m function
%   6) Produces plots comparing the uncompensated and compensated force
%      signals at the desired frequency (FP1 only)
%   7) Calculates the 2D inverse kinematics from walking data.  Adds
%      uncompensated and compensated errors to the original walking data
%      and plots the hip, knee, and ankle torques for all conditions (FP1)
%   8) Produces a plot of the RMS before and after compensation (Fxyz,Mxyz)
%      as a function of the range of cutoff frequencies (only if the range
%      is greater than one), (FP1 only)
%   9) Displays data table of the RMS before and after compensation and
%      the percent difference (FP1 only)
%   10)Performs validation tests using 10 trials of 5 different movement
%      types.  Using a calibration trial from each movement type, the 
%      inertial artifacts from all other trials are compensated with the
%      coefficients determined from the calibration trial.  A table of the 
%      mean and standard deviation of RMS values are generated using 5 
%      different calibration matrices are generated and saved as an excel
%      sheet
%======================================================================
% 
% clc
% clear
% close all

function [GRF_cor] = DoPitchSwayComp(time_cal, time_cor, GRF_cal, GRF_cor,...
                                     Acc_cal, Acc_cor, marker_cor)
%display('Starting Computation...')

%=======================================================================
%PART 1: White Noise Trials, Sensitivity Analysis, Cutoff Frequency
%=======================================================================

%-----------------------------------------------------------------------
%Loading Data
%-----------------------------------------------------------------------
%    display('Loading Data Files...')

    %Parsing Data
    Acce_cor_rotate = [-Acc_cor(:, 3), -Acc_cor(:, 1), Acc_cor(:, 2), ...
                 Acc_cor(:, 6), -Acc_cor(:, 4), -Acc_cor(:, 5)];
             
    Acce_cal_rotate = [-Acc_cal(:, 3), -Acc_cal(:, 1), Acc_cal(:, 2), ...
                 Acc_cal(:, 6), -Acc_cal(:, 4), -Acc_cal(:, 5)]; 
             
    data_cal = [time_cal, GRF_cal, Acce_cal_rotate];
    data_cor = [time_cor, GRF_cor, Acce_cor_rotate];

%-----------------------------------------------------------------------
%Filtering, Compensating, Coordinate Transformation
%-----------------------------------------------------------------------
    %Set Range of Cutoff Frequencies
    frequencies= 16;
    %Begin Compensation
    for i=1:length(frequencies)
    %Filtering
        [num,den]=butter(2,frequencies(i)/(100/2));
        data_filt_cal=filtfilt(num,den,data_cal(:,2:end));
        data_filt_cor=filtfilt(num,den,data_cor(:,2:end));
    %Clipping
        data_filt_cal=[data_cal(100:end-100,1) data_filt_cal(100:end-100,:)];
        data_filt_cor=[data_cor(100:end-100,1) data_filt_cor(100:end-100,:)];
    %---------------------------------------------------------
    %Adding Signal Delay
    %---------------------------------------------------------
        f_delay=0;
        a_delay=0;
    %---------------------
    %Adding Signal Delays  
    %---------------------
        f_cal = interp1(data_filt_cal(:,1),data_filt_cal(:,2:7),data_filt_cal(:,1)+f_delay, 'linear', 'extrap');
        a_cal = interp1(data_filt_cal(:,1),data_filt_cal(:,8:end),data_filt_cal(:,1)+a_delay, 'linear', 'extrap');

        f_cor = interp1(data_filt_cor(:,1),data_filt_cor(:,2:7),data_filt_cor(:,1)+f_delay, 'linear', 'extrap');
        a_cor = interp1(data_filt_cor(:,1),data_filt_cor(:,8:end),data_filt_cor(:,1)+a_delay, 'linear', 'extrap');

    %---------------------------------------------------------   
    %Compensating and Coordinate Transformation
    %---------------------------------------------------------
        [compensated_forces] = compensation_PSC(f_cal,a_cal,f_cor,a_cor); 
        rotated_forces = transformation(marker_cor(100:end-100, :), compensated_forces);
        
        GRF_cor(100:end-100, :) = rotated_forces;
        
    end

end
