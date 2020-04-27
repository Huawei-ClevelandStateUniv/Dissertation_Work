function [moment_cal_filt,acc_cal_filt]=...
         filter_data(data_table, cutoff_frequency)

%=========================================================================
%function FILTER_DATA
%      Filters the data through a low-pass Butterworth filter based on the
%      provided cutoff frequency.
%
%-------
%Inputs
%-------
%     data_table  (Nsamples x 4)  Array of pitch moment and
%                                 accelerations from both the
%                                 calibration and correction trials in
%                                 the form:
%                                 [Mom_cal, Mom_cor, Acc_cal, Acc_cor]
%     cutoff_frequency (double)   The desired cutoff frequency for the
%                                 low-pass filter.
%
%--------
%Outputs
%--------
%     moment_cal_filt   (Nsamples x 1)  Filtered pitch moment (cal)
%     moment_cor_filt   (Nsamples x 1)  Filtered pitch moment (cor)
%     acc_cal_filt      (Nsamples x 1)  Filtered acceleration (cal)
%     acc_cor_filt      (Nsamples x 1)  Filtered acceleration (cor)
%=========================================================================
    %Filtering
        [num,den]=butter(2,cutoff_frequency/(100/2));
        filtered=filtfilt(num,den,data_table);
        moment_cal_filt=filtered(:,1);
        acc_cal_filt=filtered(:,2);
end
