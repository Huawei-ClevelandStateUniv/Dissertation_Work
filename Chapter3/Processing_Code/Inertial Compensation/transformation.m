function [rotated_forces]=transformation(markers,forces)
                                      
%=========================================================================
%function TRANSFORMATION
%       Rotates force vectors into a global reference frame using the
%       soderquist method
%
%--------
%Inputs:
%--------
% markers    (Nsamples x 15)        XYZ positions of 5 reference plane
%                                   markers
% forces     (Nsamples x 12)        3D force plate data (forces/moments)
%                                   for both force plates in the form:
%                                   [FP1XYZ MP1XYZ FP2XYZ MP2XYZ]
%--------
%Outputs:
%--------
% rotated_forces   (Nsamples x 12)  Rotated 3D force plate data
%                                   (forces/moments) for both force
%                                   plates in the form:
%                                   [FP1XYZ MP1XYZ FP2XYZ MP2XYZ]
%=========================================================================

%-------------------------------------------------------------------------
%Making soder.m and mmat.m Available to this Function
%-------------------------------------------------------------------------
    path_to_this_file = mfilename('fullpath');
    [directory_of_this_file, ~, ~] = fileparts(path_to_this_file);
    addpath([directory_of_this_file filesep 'soder'])
%-------------------------------------------------------------------------
%Rearranging Data
%-------------------------------------------------------------------------
    %Initial Reference Coordinate Position
        x=[markers(1,1:3); markers(1,4:6); markers(1,7:9);...
           markers(1,10:12); markers(1,13:15)];
    %Redefining Variables
        FP1_forces=forces(:,1:3); FP1_moments=forces(:,4:6);
        %FP2_forces=forces(:,7:9); FP2_moments=forces(:,10:12);
    %Reshaping Matrices for soder.m 
        FP1_forces=reshape(FP1_forces(:,1:3)',3,1,length(forces));
        FP1_moments=reshape(FP1_moments(:,1:3)',3,1,length(forces));
        %FP2_forces=reshape(FP2_forces(:,1:3)',3,1,length(forces));
        %FP2_moments=reshape(FP2_moments(:,1:3)',3,1,length(forces));
%-------------------------------------------------------------------------
%Rotating Force and Moment Vectors
%-------------------------------------------------------------------------
    %Preallocating
        R=zeros(3,3,length(forces)); xpos=zeros(3,1,length(forces));
    %Calculating R and X matrices
        for i=1:length(forces)
             y=[markers(i,1:3); markers(i,4:6); markers(i,7:9);...
                markers(i,10:12); markers(i,13:15)];
            [R1,xpos1]=soder(x,y);
             R(:,:,i)=R1;
            xpos(:,:,i)=xpos1;
        end
    %Rotating the Force and Moment Vectors
        FP1=mmat(R,FP1_forces);
        MP1=cross(xpos,FP1,1)+mmat(R,FP1_moments);
        %FP2=mmat(R,FP2_forces);
        %MP2=cross(xpos,FP2,1)+mmat(R,FP2_moments);
%-------------------------------------------------------------------------
%Rearranging Matrices for Output
%-------------------------------------------------------------------------
        rotated_forces=[reshape(FP1,3,length(forces))',...
                        reshape(MP1,3,length(forces))'];
                        %reshape(FP2,3,length(forces))',...
                        %reshape(MP2,3,length(forces))'];
end 