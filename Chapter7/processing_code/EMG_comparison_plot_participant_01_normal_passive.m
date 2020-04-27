%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code generate the comparison plot of EMGs between normal walking 
% and with the passive Indego

% The third trial in the 'passive_walking' folder was the normal walking trial
% the first trial in the 'swing_control' folder was with the passive Indego
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clc
clear

passive_data_path = '..\Participant_01\Cortex\passive_walking\';
swing_data_path = '..\Participant_01\Cortex\swing_control\';

passive_trials = 3;
swing_trials = 1;

passive_EMG_mean_step = zeros(500, 6, passive_trials);
swing_EMG_mean_step = zeros(500, 6, swing_trials);

passive_EMG_std_step = zeros(500, 6, passive_trials);
swing_EMG_std_step = zeros(500, 6, swing_trials);

for trial = 1:passive_trials
    passive_EMG_mean_step(:, :, trial) = importdata(strcat(passive_data_path, 'Processed_EMG\EMG_Mean_step', num2str(trial), '.txt'));
    passive_EMG_std_step(:, :, trial) = importdata(strcat(passive_data_path, 'Processed_EMG\EMG_std_step', num2str(trial), '.txt'));
end

for trial = 1:swing_trials
    swing_EMG_mean_step(:, :, trial) = importdata(strcat(swing_data_path, 'Processed_EMG\EMG_Mean_step', num2str(trial), '.txt'));
    swing_EMG_std_step(:, :, trial) = importdata(strcat(swing_data_path, 'Processed_EMG\EMG_std_step', num2str(trial), '.txt'));
end

muscle_names = {'Biceps Femoris', 'Gluteus Maximus', 'Semitendinosus', ...
                'Lateral Gastrocnemius', 'Medium Gastrocnemius', 'Rectus Femoris'};

% muscle order in normal walking: 'Rectus Femoris', 'Vastus lateral', 'Biceps Femoris',
% 'Semitendinous', 'Lateral Gastrocnemius', 'Tibialis'          
            
figure()
phase_normal = linspace(0, 1, 500)';

subplot(3, 2, 1);  % 'Biceps Femoris'
errorfill(phase_normal', passive_EMG_mean_step(:, 3, passive_trials)', passive_EMG_std_step(:, 3, passive_trials)', 'r-', 1);
hold on;
errorfill(phase_normal', swing_EMG_mean_step(:, 1, swing_trials)',...
                         swing_EMG_std_step(:, 1, swing_trials)', 'b-', 0.6);
                     
max_signal1 = max(passive_EMG_mean_step(:, 3, passive_trials)' ...
               + passive_EMG_std_step(:, 3, passive_trials)');

max_signal2 = max(swing_EMG_mean_step(:, 1, swing_trials)' ...
               + swing_EMG_std_step(:, 1, swing_trials)');

max_signal = max(max_signal1, max_signal2);

hold on    
plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
hold on
plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
title(muscle_names{1});
axis([0, 1, 0, 1.2*max_signal]);
hold off

subplot(3, 2, 2);  % 'Gluteus Maximus'
errorfill(phase_normal', swing_EMG_mean_step(:, 2, swing_trials)',...
                         swing_EMG_std_step(:, 2, swing_trials)', 'b-', 0.6);
                     
max_signal2 = max(swing_EMG_mean_step(:, 2, swing_trials)' ...
               + swing_EMG_std_step(:, 2, swing_trials)');

max_signal = max_signal2;

hold on    
plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
hold on
plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
title(muscle_names{2});
axis([0, 1, 0, 1.2*max_signal]);
hold off

%sgtitle('Muscle Activation in Step (Constant Speed)');
% 'Normal Walking std','Normal Walking mean',
legend('Normal Walking std', ...
       'Normal Walking mean', 'Passive Walking std', 'Passive Walking mean')



subplot(3, 2, 3);  % 'Semitendinosus'
errorfill(phase_normal', passive_EMG_mean_step(:, 4, passive_trials)',...
                         passive_EMG_std_step(:, 4, passive_trials)', 'r-', 1);
hold on;
errorfill(phase_normal', swing_EMG_mean_step(:, 3, swing_trials)',...
                         swing_EMG_std_step(:, 3, swing_trials)', 'b-', 0.6);
                     
max_signal1 = max(passive_EMG_mean_step(:, 4, passive_trials)' ...
               + passive_EMG_std_step(:, 4, passive_trials)');

max_signal2 = max(swing_EMG_mean_step(:, 3, swing_trials)' ...
               + swing_EMG_std_step(:, 3, swing_trials)');

max_signal = max(max_signal1, max_signal2);

hold on    
plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
hold on
plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
title(muscle_names{3});
axis([0, 1, 0, 1.2*max_signal]);
hold off

subplot(3, 2, 4);  % 'Lateral Gastrocnemius'
errorfill(phase_normal', passive_EMG_mean_step(:, 5, passive_trials)',...
                         passive_EMG_std_step(:, 5, passive_trials)', 'r-', 1);
hold on;
errorfill(phase_normal', swing_EMG_mean_step(:, 4, swing_trials)',...
                         swing_EMG_std_step(:, 4, swing_trials)', 'b-', 0.6);
                     
max_signal1 = max(passive_EMG_mean_step(:, 5, passive_trials)' ...
               + passive_EMG_std_step(:, 5, passive_trials)');

max_signal2 = max(swing_EMG_mean_step(:, 4, swing_trials)' ...
               + swing_EMG_std_step(:, 4, swing_trials)');

max_signal = max(max_signal1, max_signal2);

hold on    
plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
hold on
plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
title(muscle_names{4});
axis([0, 1, 0, 1.2*max_signal]);
hold off

subplot(3, 2, 5);  % 'Gluteus Maximus'
errorfill(phase_normal', swing_EMG_mean_step(:, 5, swing_trials)',...
                         swing_EMG_std_step(:, 5, swing_trials)', 'b-', 0.6);
                     
max_signal2 = max(swing_EMG_mean_step(:, 5, swing_trials)' ...
               + swing_EMG_std_step(:, 5, swing_trials)');

max_signal = max_signal2;

hold on    
plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
hold on
plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
title(muscle_names{5});
axis([0, 1, 0, 1.2*max_signal]);
xlabel('Gait Phase')
hold off

subplot(3, 2, 6);  % 'Gluteus Maximus'
errorfill(phase_normal', swing_EMG_mean_step(:, 6, swing_trials)',...
                         swing_EMG_std_step(:, 6, swing_trials)', 'r-', 0.6);
hold on
errorfill(phase_normal', swing_EMG_mean_step(:, 6, swing_trials)',...
                         swing_EMG_std_step(:, 6, swing_trials)', 'b-', 0.6);
                     
max_signal2 = max(swing_EMG_mean_step(:, 6, swing_trials)' ...
               + swing_EMG_std_step(:, 6, swing_trials)');

max_signal = max_signal2;

hold on    
plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
hold on
plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
title(muscle_names{6});
axis([0, 1, 0, 1.2*max_signal]);
xlabel('Gait Phase')
hold off

