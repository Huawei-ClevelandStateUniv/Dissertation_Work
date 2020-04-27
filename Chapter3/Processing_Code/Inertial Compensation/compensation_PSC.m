function [compensated_forces]=compensation_PSC(force_cal,accel_cal,...
                                           force_cor,accel_cor) 
                                      
%=========================================================================
%function COMPENSATION:
%   1)Compensates for the inertia of a moving instrumented treadmill, 
%     assuming a linear relationship between force plate signals and 
%     accelerometers
%   2)Generates a calibration matrix of coefficients based on linear least
%     squares regression between forces (B) and accelerations (D) of an
%     unweighted treadmill platform under prescribed movements
%   3)Corrects force signals of another trial under similar movements
%     by applying the calibration matrix and subtracting inertial artifacts
%
%--------
%Inputs:
%--------
%  ~~Calibration Data~~
%      force_cal    (Nsamples x 12)   3D force plate data (forces/moments)
%                                      for both force plates in the form:
%                                      [FP1XYZ MP1XYZ FP2XYZ MP2XYZ]
%      accel_cal    (Nsamples x 12)   3D accelerations from 4 
%                                      accelerometers in the form:
%                                      [A1XYZ A2XYZ A3XYZ A4XYZ]
%  ~~Correction Data~~
%      force_cor    (Nsamples x 12)   3D force plate data (forces/moments)
%                                      for both force plates in the form: 
%                                      [FP1XYZ MP1XYZ FP2XYZ MP2XYZ]
%      accel_cor (Nsamples x 12)   3D accelerations from 4 
%                                      accelerometers in the form:
%                                      [A1XYZ A2XYZ A3XYZ A4XYZ]
%--------
%Outputs:
%--------
%      compensated_forces (Nsamples x 12)   Compensated 3D force plate data 
%                                           (forces/moments) for both force 
%                                           plates in the form:
%                                           [FP1XYZ MP1XYZ FP2XYZ MP2XYZ]
%=========================================================================

%-------------------------------------------------------------------------
%Determining Calibration Coefficients
%-------------------------------------------------------------------------
    %-------------------------
    %Acceleration (D) Matrix
    %-------------------------
    
        a_more=ones(length(force_cal),1);
        accel_cal=[accel_cal a_more];
        [~, colnum] = size(accel_cal);
        D=zeros(6*length(force_cal),colnum*6);
            for i=1:length(force_cal)
                for j=1:6
                    row=6*(i-1)+j;
                    col=colnum*(j-1)+(1:colnum);
                    D(row,col)=accel_cal(i,:);
                end
            end
    %-------------------------
    %Force Matrix (B)
    %-------------------------
        FP1_cal=force_cal(:,1:6);
        %FP2_cal=force_cal(:,7:12);
        %Reshaping for Least Squares
            B1=reshape(FP1_cal',6*length(force_cal),1);
            %B2=reshape(FP2_cal',6*length(force_cal),1);
    %---------------------------------------
    %Calculate Coefficients (Least Squares)
    %---------------------------------------
        C_FP1=D\B1;
        %C_FP2=D\B2;
%-------------------------------------------------------------------------
%Correcting Force Plate Signals Using Calibration Matrices
%-------------------------------------------------------------------------
    %---------------------------
    %Acceleration (D) Matrix
    %---------------------------
        a_more=ones(length(force_cor),1);
        accel_cor=[accel_cor a_more];
        [~, colnum] = size(accel_cor);
        D=zeros(6*length(force_cor),colnum*6);
            for i=1:length(force_cor)
                for j=1:6
                    row=6*(i-1)+j;
                    col=colnum*(j-1)+(1:colnum);
                    D(row,col)=accel_cor(i,:);
                end
            end
    %----------------------------
    %Force Matrix (B)Generation
    %----------------------------
        FP1_cor=force_cor(:,1:6);
        %FP2_cor=force_cor(:,7:12);
            %Reshaping for Least Squares
            B1=reshape(FP1_cor',6*length(force_cor),1);
            %B2=reshape(FP2_cor',6*length(force_cor),1);
    %-----------------------------------------------------------------------
    %Correcting Forces using Calibration Matrices (C_FP1, C_FP2)
    %-----------------------------------------------------------------------
        FP1_corrected=B1-(D*C_FP1);
        %=B2-(D*C_FP2);
        %Reshaping and Generating Output
            %FP1_corrected=reshape(FP1_corrected,6,length(force_cor));
            FP1_corrected=reshape(FP1_corrected,6,length(force_cor))';
            %FP2_corrected=reshape(FP2_corrected,6,length(force_cor));
            %FP2_corrected=reshape(FP2_corrected,6,length(force_cor))';
            compensated_forces=FP1_corrected; 
end 