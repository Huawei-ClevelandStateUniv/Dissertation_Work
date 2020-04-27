%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the main code to process the raw data measured in the standing
% balance experiment. General steps are:

% 1. Fill the gaps in the raw marker data
% 2. Inertia compensation of the grf in perturbation trials
% 3. Calculate joint angles and torques based on the gap filled marker data
%    and joint motion and
%    the ground reaction forces.
%	 Joints: hip, knee, ankle
%    Sign convention for angles and moments: hip flexion, knee extension,
%    ankle Dorsiflexion are positive
% 4. Calculate model parameters based on segment length, height and weight 
% 5. Statistical analysis of the calculated joint motion
% 6. Save processed data into files

% Cooresponding data will be write to the 'processing_data' folder.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;

num_subj = 8;           % total number of subject
num_trial = 4;          % total number of experimental trial of each subject
num_markers = 32;       % total number of mocap markers in each trial
num_joints = 3;         % total number of joints in the leg2d model

% Measured height (cm), weight, knee width, and ankle width for subject 3-8
par_info = [180.34, 79.12, 11.75, 7.4; ...  
            178.00, 63.10, 9.65, 6.75; ...
            179.00, 70.56, 12.05, 7.45; ...
            165.00, 58.24, 10.70, 7.05; ...
            175.26, 68.75, 10.70, 6.95; ...
            163.00, 60.33, 10.75, 6.90;];

% specify data folder
raw_data_folder = '..\Raw_Data\';
processed_data_folder = '..\Processed_Data\';


%% load unloaded trial data: grf and acceleration
unloaded_marker_file_name = strcat(raw_data_folder, 'Mocap0000.txt');
unloaded_record_file_name = strcat(raw_data_folder, 'Record0000.txt');

unloaded_marker_data = importdata(unloaded_marker_file_name);
unloaded_analog_F1 = unloaded_marker_data.data(:, 36:41);

time_unloaded = (0:0.01:(length(unloaded_marker_data.data(:, 1))-1)*0.01)';
                  
unloaded_F1 = unloaded_marker_data.data(:, 21:26);
unloaded_F1(:, 5) = RegainYm_Unloaded(unloaded_analog_F1);

unloaded_record_data = importdata(unloaded_record_file_name);
unloaded_accel = interp1(unloaded_record_data.data(:, 1) - unloaded_record_data.data(1, 1),...
                  unloaded_record_data.data(:, 3:end), time_unloaded, 'linear', 'extrap');

%% process data
for subj = 3:num_subj  % only process last six participants
    
    fprintf('Processing Participant Number %d\n', subj)
    
    % create process data folder
    processed_folder = strcat(processed_data_folder, 'Subj0', ...
                                            num2str(subj));                      
    [status, msg, msgID] = mkdir(processed_folder);
    
    figure()
    
    for trial = 1:num_trial
        
        % specify data name and path
        raw_marker_file_name = strcat(raw_data_folder, 'Subj0', num2str(subj), ...
                                    '\Mocap000', num2str(trial), '.txt');
        
        raw_record_file_name = strcat(raw_data_folder, 'Subj0', num2str(subj), ...
                                    '\Record000', num2str(trial), '.txt');
                                
        process_mocap_file_name = strcat(processed_folder, '\Mocap000', ...
                                            num2str(trial), '.txt');
                                        
        process_motion_file_name = strcat(processed_folder, '\Motion000', ...
                                            num2str(trial), '.txt');
                                        
        data_info_save_file_name = strcat(processed_folder, '\Data_info000', ...
                                        num2str(trial), '.txt');
                                    
        motion_analysis_fig_name = strcat(processed_folder, '\MotionAnalysis.fig');
                                    
        raw_marker_data = importdata(raw_marker_file_name);
        raw_record_data = importdata(raw_record_file_name);
        
        % 1. Fill the gaps in marker data and generate the gap information.
        [filled_marker_data, gap_info] = FillMarkerGap(raw_marker_data.data(:,...
                                                       1:num_markers*3 + 2));
        
        % 2. Apply intertia compensation of the recorded ground reation
        % forces        
        loaded_analog_F1 = raw_marker_data.data(:, 117:122);
        loaded_F1 = raw_marker_data.data(:, 102:107);
        loaded_F1(:, 5) = RegainYm(loaded_analog_F1);
        
        time_loaded = (0:0.01:(length(raw_marker_data.data(:, 1))-1)*0.01)';
        
        if trial > 1 && trial < 4  % do not apply compensation for the quiet standing trials
            
            loaded_accel = interp1(raw_record_data.data(:, 1) - raw_record_data.data(1, 1), ...
                           raw_record_data.data(:, 3:end), time_loaded, 'linear', 'extrap');

            loaded_treadmill_marker = filled_marker_data(:, end-14:end);

            grf_compensated = DoPitchSwayComp(time_unloaded, time_loaded, unloaded_F1,...
                                        loaded_F1, unloaded_accel, loaded_accel,...
                                        loaded_treadmill_marker);
        else
           
            grf_compensated = loaded_F1;
            
        end
                                
        grf_comp_percentage = 1 - std(grf_compensated, 1)./std(loaded_F1, 1);
         
        % 3. Cacluate the joint angles and torques using the leg2d model (Ton van den Bogert).
        
        % Coordinate system:
        %	X is forward (direction of walking), Y is up
        %
        % Markers:
        %	1: Shoulder
        %	2: Greater trochanter
        %	3: Lateral epicondyle of knee
        %	4: Lateral malleolus
        %	5: Heel (placed at same height as marker 6)
        %	6: Head of 5th metatarsal
        
        Lmarkers_data = [-filled_marker_data(:, 78), filled_marker_data(:, 79),...
                         -filled_marker_data(:, 30), filled_marker_data(:, 31),...
                         -filled_marker_data(:, 36), filled_marker_data(:, 37),...
                         -filled_marker_data(:, 42), filled_marker_data(:, 43),...
                         -filled_marker_data(:, 45), filled_marker_data(:, 46),...
                         -filled_marker_data(:, 51), filled_marker_data(:, 52)];
                     
        Rmarkers_data = [-filled_marker_data(:, 81), filled_marker_data(:, 82), ...
                         -filled_marker_data(:, 54), filled_marker_data(:, 55), ...
                         -filled_marker_data(:, 60), filled_marker_data(:, 61), ...
                         -filled_marker_data(:, 66), filled_marker_data(:, 67), ...
                         -filled_marker_data(:, 69), filled_marker_data(:, 70), ...
                         -filled_marker_data(:, 75), filled_marker_data(:, 76)];
                     
        marker_data = (Lmarkers_data + Rmarkers_data)/2;
        grf_data = [-grf_compensated(:, 1), grf_compensated(:, 2), -grf_compensated(:, 6)];
        
        options.freq = 16;
        
        [angles, velocities, moments, forces] = leg2d(time_loaded, marker_data,...
                                                      grf_data/par_info(subj-2, 2),...
                                                      options);
        % Calculate absolute joint motions by substracting the quiet
        % standing joint angles (2 - 8 seconds). The assumption is that human 
        % trends to save engergy in quiet standing, so that the joint angles 
        % should be close to zero which requires the minimum joint torques.
        % In addition, the purpose of this data set is for identifying
        % the postural feedback controllers that can be used in humanoid robots
        % and P/O devices, it is better to have zero joint angle at quiet
        % standing, so that less joint torques will be required.
        
        for j = 1:num_joints
            angles(:, j) = angles(:, j) - mean(angles(200:800, j));
            moments(:, j) = moments(:, j) - mean(moments(200:800, j));
        end
        
        % change the positive sign of knee angle as anti-clockwise
        angles(:, 2) = -angles(:, 2);
        velocities(:, 2) = -velocities(:, 2);
        moments(:, 2) = -moments(:, 2);
        
        angles(:, 3) = -angles(:, 3);
        velocities(:, 3) = -velocities(:, 3);
        moments(:, 3) = -moments(:, 3);
                 
        moments = moments*par_info(subj-2, 2);
        forces = forces*par_info(subj-2, 2);
        
        % 4. Calcualte model parameters
        if trial == 1
            Parameters = Model_Parameter_Calculate(filled_marker_data,...
                                    par_info(subj-2, 2), par_info(subj-2, 1),...
                                    par_info(subj-2, 3), par_info(subj-2, 4));
                                
            
        end
        
        % 5. statistical analysis
        
        MeanJ = zeros(num_joints, 1);
        STDJ = zeros(num_joints, 1);
        
        for m = 1:num_joints
            
            MeanJ(m) = mean(angles(2000:end, m));
            STDJ(m) = std(angles(2000:end, m));  
           
        end
        % plot the statistical results of each subject
        subplot(1, num_trial, trial)
        errorbar([1, 2, 3], MeanJ*180/pi, STDJ*180/pi, '.', 'Linewidth',...
                                                    1.5, 'MarkerSize', 16)
        hold on
        title(strcat('Subj0', num2str(subj), 'Trial', num2str(trial)))
        xticks([1, 2, 3])
        xticklabels({'Hip', 'Knee', 'Ankle'})
        axis([0.5, 3.5, -20, 20])
        ylabel('Mean + Std (deg.)')
       
        % 6. Save processed data
        
        process_mocap = [filled_marker_data, grf_compensated];
        process_motion = [time_loaded, angles, velocities, moments, forces];
        
        % save processed marker data
        fid=fopen(process_mocap_file_name,'w');   % open a new file for writing
        headline = raw_marker_data.textdata([1:2+3*num_markers, 2+3*num_markers+4:2+3*num_markers+9]);
            for ix=1:length(headline)
                fprintf(fid,'%s    ', char(headline{1, ix}));  % write the header lines
            end             
                fprintf(fid, '\n');

            for iy = 1:length(process_mocap(:, 1))
                fprintf(fid,'%d ', process_mocap(iy, :)');  % then the data
                fprintf(fid, '\n');
            end

        fclose(fid);
        
        % save the processed motion/moment data
        fid1 = fopen(process_motion_file_name, 'w');  % open a new file for writing
            
        headline1 = {'Time', 'HipA', 'KneeA', 'AnkleA', 'HipV', 'KneeV', 'AnkleV',...
                'HipM', 'KneeM', 'AnkleM', 'HipFx', 'HipFy', 'KneeFx', 'KneeFy',...
                'AnkleFx', 'AnkleFy'};
        
        for ix=1:length(headline1)
            fprintf(fid1,'%s    ', char(headline1{1, ix}));  % write the header lines
        end             
            fprintf(fid1, '\n');

        for iy = 1:length(process_motion(:, 1))
            fprintf(fid1,'%d ', process_motion(iy, :)');  % then the data
            fprintf(fid1, '\n');
        end
        fclose(fid1);
        
        % save the data quality information data
        fid2 = fopen(data_info_save_file_name, 'w');  % open a new file for writing
        
        head_description1 = strcat('Quality of marker data. First row is the',...
                                'maximum gap; second row is the percentage of',...
                                'missing data frames');
        
        fprintf(fid2,'%s   \n\n', head_description1);  % write the header lines
            
        headline21 = raw_marker_data.textdata(3:2+3*num_markers);
        
        for ix=1:length(headline21)
            fprintf(fid2,'%s    ', char(headline21{1, ix}));  % write the header lines
        end             
            fprintf(fid2, '\n');

        for iy = 1:length(gap_info(:, 1))
            fprintf(fid2,'%d ', gap_info(iy, :)');  % then the data
            fprintf(fid2, '\n');
        end
        
        head_description2 = strcat('Precentage of compensated GRF');
        
        fprintf(fid2,'\n %s   \n\n', head_description2);  % write the header lines        
        
        headline22 = raw_marker_data.textdata(2+3*num_markers+3:2+3*num_markers+9);
        
        for ix=1:length(headline22)
            fprintf(fid2,'%s    ', char(headline22{1, ix}));  % write the header lines
        end             
            fprintf(fid2, '\n');

        for iy = 1:length(grf_comp_percentage)
            fprintf(fid2,'%d ', grf_comp_percentage(iy));  % then the data
            fprintf(fid2, '\n');
        end
        
        fclose(fid2);
          
    end
    
    savefig(motion_analysis_fig_name);
    hold off
    
end

%% Plot joint motion/moment and raw/compenated GRF of an example experimental trial

example_subj = 7;
example_trial = 3;

fprintf('Check processing result, taking subject %d and trial %d as example \n', example_subj, example_trial)

% load data
raw_data = importdata(strcat(raw_data_folder, 'Subj0', num2str(example_subj),...
                             '/Mocap000', num2str(example_trial) ,'.txt'));
processed_data = importdata(strcat(processed_data_folder, 'Subj0', num2str(example_subj),...
                                   '/Mocap000', num2str(example_trial) ,'.txt'));
processed_motion = importdata(strcat(processed_data_folder, 'Subj0', num2str(example_subj),...
                                     '/Motion000', num2str(example_trial) ,'.txt'));

raw_analog_F1 = raw_data.data(:, 117:122);
raw_F1 = raw_data.data(:, 102:107);
raw_F1(:, 5) = RegainYm(raw_analog_F1);

compensated_F1 = processed_data.data(:, end-5:end);

% plot motion/moment
figure()
subplot(2,1,1)
plot(processed_motion.data(:, 1), processed_motion.data(:, 2)*180/pi)
hold on
plot(processed_motion.data(:, 1), processed_motion.data(:, 3)*180/pi)
hold on
plot(processed_motion.data(:, 1), processed_motion.data(:, 4)*180/pi)
hold off
legend('Hip', 'Knee', 'Ankle')
ylabel('Joint motion (deg.)')
title(strcat('Motion - Moment - Subj0', num2str(example_subj), '- Trial0', num2str(example_trial)))

subplot(2,1,2)
plot(processed_motion.data(:, 1), processed_motion.data(:, 8))
hold on
plot(processed_motion.data(:, 1), processed_motion.data(:, 9))
hold on
plot(processed_motion.data(:, 1), processed_motion.data(:, 10))
hold off
legend('Hip', 'Knee', 'Ankle')
ylabel('Joint moment (Nm)')
xlabel('Time (s)')

savefig(strcat(processed_data_folder, 'Example_Motion_Moment.fig'))

% plot raw and compensated ground reaction force
figure()
subplot(3,1,1)
plot(processed_motion.data(:, 1), -raw_F1(:, 1))
hold on
plot(processed_motion.data(:, 1), -compensated_F1(:, 1))
hold off
legend('raw', 'compensated')
ylabel('Fx (N)')
title(strcat('GRF - Subj0', num2str(example_subj), '- Trial0', num2str(example_trial)))

subplot(3,1,2)
plot(processed_motion.data(:, 1), raw_F1(:, 2))
hold on
plot(processed_motion.data(:, 1), compensated_F1(:, 2))
hold off
legend('raw', 'compensated')
ylabel('Fy (N)')

subplot(3,1,3)
plot(processed_motion.data(:, 1), -raw_F1(:, 6))
hold on
plot(processed_motion.data(:, 1), -compensated_F1(:, 6))
hold off
legend('raw', 'compensated')
ylabel('Mz (Nm)')

savefig(strcat(processed_data_folder, 'Example_GRF_raw_compensated.fig'))



