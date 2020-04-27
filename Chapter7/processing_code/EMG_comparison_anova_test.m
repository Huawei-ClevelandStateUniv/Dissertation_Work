clc
clear
close all

data_path = '..\Participant_01\Cortex\swing_control\';
trial1 = 4;
trial2 = 5;

right_step_index_1 = load(strcat(data_path, 'EMG_hs_to_index_right_trial0', num2str(trial1),'.txt'));
analog_data_1 = importdata(strcat(data_path, 'Mocap000', ...
                        num2str(trial1) ,'_Motion Analysis_analog.txt'));
                   
right_step_index_2 = load(strcat(data_path, 'EMG_hs_to_index_right_trial0', num2str(trial2),'.txt'));
analog_data_2 = importdata(strcat(data_path, 'Mocap000', ...
                        num2str(trial2) ,'_Motion Analysis_analog.txt'));

EMG_1 = analog_data_1.data(:, [17, 21, 25, 29, 33, 41] + 2);
EMG_2 = analog_data_2.data(:, [17, 21, 25, 29, 33, 41] + 2);

[b1, a1] = butter(2, 25/(1000/2), 'high');
[b2, a2] = butter(2, 4/(1000/2), 'low');

EMG1_1 = filtfilt(b1, a1, EMG_1); % High pass filter, 25Hz
EMG2_1 = abs(EMG1_1); % Rectify
EMG3_1 = filtfilt(b2, a2, EMG2_1); % Low pass filter, 6Hz

EMG1_2 = filtfilt(b1, a1, EMG_2); % High pass filter, 25Hz
EMG2_2 = abs(EMG1_2); % Rectify
EMG3_2 = filtfilt(b2, a2, EMG2_2); % Low pass filter, 6Hz

mkdir(strcat(data_path, 'Processed_EMG'))

%% normalize EMG in the along the step 

skip_num = 15;

EMG_averg_step_1 = zeros(500, 6, length(right_step_index_1(:, 2)) - skip_num - 1);
EMG_averg_step_2 = zeros(500, 6, length(right_step_index_2(:, 2)) - skip_num - 1);


for s = skip_num : length(right_step_index_1(:, 2))-1
    
    EMG_step = EMG3_1(right_step_index_1(s, 1)+48:right_step_index_1(s, 3)+48 ,:);  %EMG delay 48 ms
    time_analog = (0:0.001:(right_step_index_1(s, 3) - right_step_index_1(s, 1))*0.001)';
    time_normal = linspace(0, (right_step_index_1(s, 3) - right_step_index_1(s, 1))*0.001, 500)';
    EMG_averg_step_1(:, :, s - skip_num + 1) = interp1(time_analog, EMG_step, time_normal, 'linear', 'extrap');  
    
end

for s = skip_num:length(right_step_index_2(:, 2))-1
    
    EMG_step = EMG3_2(right_step_index_2(s, 1)+48:right_step_index_2(s, 3)+48 ,:);  %EMG delay 48 ms
    time_analog = (0:0.001:(right_step_index_2(s, 3) - right_step_index_2(s, 1))*0.001)';
    time_normal = linspace(0, (right_step_index_2(s, 3) - right_step_index_2(s, 1))*0.001, 500)';
    EMG_averg_step_2(:, :, s - skip_num + 1) = interp1(time_analog, EMG_step, time_normal, 'linear', 'extrap');  
    
end

%% one way anova test
P_value = zeros(500, 6);

for i = 1:500
    for j = 1:6
        
        y = [squeeze(EMG_averg_step_1(i, j, :)); squeeze(EMG_averg_step_2(i, j, :))];
        group = [zeros(length(EMG_averg_step_1(i, j, :)), 1); ...
                 ones(length(EMG_averg_step_2(i, j, :)), 1)];
             
        [P_value(i, j),~,~] = anova1(y, group, 'off');
       
    end
end

%% save results

fid = fopen(strcat(data_path, 'Processed_EMG\P_value_anova_test_', num2str(trial1),...
                              '_', num2str(trial2) , '.txt'), 'w');
for iy = 1:length(P_value(:, 1))
    fprintf(fid,'%d ', P_value(iy, :));  % then the data
    fprintf(fid, '\n');
end
fclose(fid);

