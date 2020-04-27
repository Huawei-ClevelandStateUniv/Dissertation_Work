function m_simulated=simulate(acc,theta)

%=========================================================================
%FUNCTION simulate
%   1) Compensates for inertial errors in pitch moment due to the
%      rotation of the treadmill rollers.
%   2) Simulates the pitch moment by using recorded accelerations and
%      the coefficients obtained from the calibration function
%      pitch_moment_calibration
%   3) Clips the first and last second from the simulated data
%
%--------
%Inputs
%--------
%   acc       (Nsamples x 1)    Column vector of recorded accelerations
%                               from the trial to be corrected
%   theta     (5 x 1)           Column vector coefficients obtained from the
%                               mx_calibration function
%--------
%Outputs
%--------
%   m_simulated ((Nsamples - 199) x 1)  Column vector of the simulated
%                                       pitch moment by applying the
%                                       coefficients obtained from the
%                                       calibration
%=========================================================================

%Defining Coefficients
    A=[1 theta(1) theta(2)];
    B=[theta(3) theta(4) theta(5)];
%Simulating
    ysim=filter(B,A,acc);
    m_simulated=ysim(100:end-100,:);
 end
