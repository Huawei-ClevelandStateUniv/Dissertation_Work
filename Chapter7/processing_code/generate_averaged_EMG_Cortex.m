clc
clear
close all

data_path = '..\Participant_02\Cortex\passive_walking\';
trials = [2];

for trial = trials

right_step_index = load(strcat(data_path, 'EMG_hs_to_index_right_trial0', num2str(trial),'.txt'));

analog_data = importdata(strcat(data_path, 'Mocap000', ...
                        num2str(trial) ,'_Motion Analysis_analog.txt'));

EMG = analog_data.data(:, [17, 21, 25, 29, 33, 41] + 2);

[b1, a1] = butter(2, 25/(1000/2), 'high');
[b2, a2] = butter(2, 4/(1000/2), 'low');

EMG1 = filtfilt(b1, a1, EMG); % High pass filter, 25Hz
EMG2 = abs(EMG1); % Rectify
EMG3 = filtfilt(b2, a2, EMG2); % Low pass filter, 6Hz

mkdir(strcat(data_path, 'Processed_EMG'))

%% Average EMG in the swing phase

EMG_averg_swing = zeros(250, 6, length(right_step_index(:, 2)));
EMG_averg_step = zeros(500, 6, length(right_step_index(:, 2)));
for s = 15:length(right_step_index(:, 2))-1
    
    %EMG delay 48 ms; heel strike also detected with around 48 ms delay. Therefore, no adjustment is needed.

    EMG_swing = EMG3(right_step_index(s, 2)+0:right_step_index(s, 3)+0 ,:);  
    time_analog = (0:0.001:(right_step_index(s, 3) - right_step_index(s, 2))*0.001)';
    time_normal = linspace(0, (right_step_index(s, 3) - right_step_index(s, 2))*0.001, 250)';
    EMG_averg_swing(:, :, s) = interp1(time_analog, EMG_swing, time_normal, 'linear', 'extrap');  
    
    EMG_step = EMG3(right_step_index(s, 1)+0:right_step_index(s, 3)+0 ,:); 
    time_analog = (0:0.001:(right_step_index(s, 3) - right_step_index(s, 1))*0.001)';
    time_normal = linspace(0, (right_step_index(s, 3) - right_step_index(s, 1))*0.001, 500)';
    EMG_averg_step(:, :, s) = interp1(time_analog, EMG_step, time_normal, 'linear', 'extrap');  
    
end

EMG_mean_swing = mean(EMG_averg_swing, 3);
EMG_std_swing = std(EMG_averg_swing, 0, 3);

EMG_mean_step = mean(EMG_averg_step, 3);
EMG_std_step = std(EMG_averg_step, 0, 3);

% fid = fopen(strcat(data_path, 'Processed_EMG\EMG_Mean_swing', num2str(trial), '.txt'), 'w');
% for iy = 1:length(EMG_mean_swing(:, 1))
%     fprintf(fid,'%d ', EMG_mean_swing(iy, :));  % then the data
%     fprintf(fid, '\n');
% end
% fclose(fid);
% 
% fid = fopen(strcat(data_path, 'Processed_EMG\EMG_std_swing', num2str(trial), '.txt'), 'w');
% 
% for iy = 1:length(EMG_std_swing(:, 1))
%     fprintf(fid,'%d ', EMG_std_swing(iy, :));  % then the data
%     fprintf(fid, '\n');
% end
% fclose(fid);
% 
% fid = fopen(strcat(data_path, 'Processed_EMG\EMG_Mean_step', num2str(trial), '.txt'), 'w');
% 
% for iy = 1:length(EMG_mean_step(:, 1))
%     fprintf(fid,'%d ', EMG_mean_step(iy, :));  % then the data
%     fprintf(fid, '\n');
% end
% fclose(fid);
% 
% fid = fopen(strcat(data_path, 'Processed_EMG\EMG_std_step', num2str(trial), '.txt'), 'w');
% 
% for iy = 1:length(EMG_std_step(:, 1))
%     fprintf(fid,'%d ', EMG_std_step(iy, :));  % then the data
%     fprintf(fid, '\n');
% end
% fclose(fid);

muscle_names = {'Biceps Femoris', 'Glutaues Maximum', 'Semitendinous', ...
                'Lateral Gastrocnemius', 'Medium Gastrocnemius', 'Rectus Femoris'};
            
% muscle_names = {'Rectus Femoris', 'Vastus lateral', 'Biceps Femoris',...
%     'Semitendinous', 'Lateral Gastrocnemius', 'Tibialis'};

figure()
title('Muscle Activation in Swing')
phase_normal = linspace(0.5, 1, 250)';
for k = 1:6
    subplot(3, 2, k);
    errorfill(phase_normal', EMG_mean_swing(:, k)', EMG_std_swing(:, k)', 'r-', 1);
    hold on;
    plot([0.5, 1], [0, 0], 'k--');
    title(muscle_names{k});
end
hold off
sgtitle('Muscle Activation in a swing period');
savefig(strcat(data_path, 'Processed_EMG\Muscle_activation_swing', num2str(trial), '.fig'));


figure()
title('Normal Walking')
phase_normal = linspace(0, 1, 500)';
for k = 1:6
    subplot(3, 2, k);
    errorfill(phase_normal', EMG_mean_step(:, k)', EMG_std_step(:, k)', 'r-', 1);
    hold on;
    plot([0, 0.6], [0, 0], 'k-', 'linewidth', 2.5);
    title(muscle_names{k});
end
hold off
sgtitle('Normal Walking');

savefig(strcat(data_path, 'Processed_EMG\Muscle_activation_step', num2str(trial), '.fig'));

end
