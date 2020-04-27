%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code generate the comparison plot of EMGs between normal walking 
% and with the passive Indego

% The first trial in the 'passive_walking' folder was the normal walking trial
% the first trial in the 'swing_control' folder was with the passive Indego
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clc
clear

passive_data_path = '..\Participant_02\Cortex\passive_walking\';
swing_data_path = '..\Participant_02\Cortex\swing_control\';

passive_trials = 4;
swing_trials = 7;

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
                'Lateral Gastrocnemius', 'Medial Gastrocnemius', 'Rectus Femoris'};

figure()
phase_normal = linspace(0, 1, 500)';

for k = 1:6
    subplot(3, 2, k);
    [~, h1] = errorfill(phase_normal', passive_EMG_mean_step(:, k, 1)', passive_EMG_std_step(:, k, 1)', 'k-', 1);
    hold on;
    [~, h2] = errorfill(phase_normal', swing_EMG_mean_step(:, k, 1)',...
                             swing_EMG_std_step(:, k, 1)', 'b-', 0.6);
    
    max_signal1 = max(passive_EMG_mean_step(:, k, 1)' ...
                   + passive_EMG_std_step(:, k, 1)');
               
    max_signal2 = max(swing_EMG_mean_step(:, k, 1)' ...
                   + swing_EMG_std_step(:, k, 1)');
               
    max_signal = max(max_signal1, max_signal2);
    
    hold on    
    plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
    hold on
    plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
    title(muscle_names{k});
    
    axis([0, 1, 0, 1.2*max_signal]);
    
    if k == 5 || k == 6
        xlabel('Gait Phase')
    end
    
end
hold off
%sgtitle('Muscle Activation in Step (Constant Speed)');
% 'Normal Walking std','Normal Walking mean',
legend([h1, h2], 'Normal Walking', 'Passive Walking')


figure()
phase_normal = linspace(0, 1, 500)';
for k = 1:6
    subplot(3, 2, k);
    [~, h1] = errorfill(phase_normal', passive_EMG_mean_step(:, k, 2)', passive_EMG_std_step(:, k, 2)', 'k-', 1.0);
    hold on;
    [~, h2] = errorfill(phase_normal', swing_EMG_mean_step(:, k, 7)',...
                             swing_EMG_std_step(:, k, 7)', 'b-', 0.6);
    hold on;
 
    max_signal1 = max(passive_EMG_mean_step(:, k, 2)' ...
                   + passive_EMG_std_step(:, k, 2)');
               
    max_signal2 = max(swing_EMG_mean_step(:, k, 7)' ...
                   + swing_EMG_std_step(:, k, 7)');
               
    max_signal = max(max_signal1, max_signal2);
               
    hold on    
    plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
    hold on
    plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
    title(muscle_names{k});
    
    if k == 5 || k == 6
        xlabel('Gait Phase')
    end
    
    axis([0, 1, 0, 1.2*max_signal]);
end
hold off
%sgtitle('Muscle Activation in Step (Perturbed Speed)');
% 'Normal Walking std','Normal Walking mean',
lgd = legend([h1, h2], 'Normal Walking', 'Passive Walking');
set(lgd,'FontSize',10);
