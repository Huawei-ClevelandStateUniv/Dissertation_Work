%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code generate the comparison plot of EMGs
% Trial 1: Passive walking
% Trial 2: Walking with right side swing control
% Trial 3: Walking with right side swing control (hip 1.2x)
% Trial 4: Walking with right side swing control (hip 1.4x)
% Trial 5: Walking with right side swing control with perturbed speed
% Trial 6: Walking with right side swing control with perturbed speed (repeat)
% Trial 7: Passive walking with perturbed speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clc
clear

passive_data_path = '..\Participant_02\Cortex\passive_walking\';
swing_data_path = '..\Participant_02\Indego\swing_control\';

passive_trials = 2;
swing_trials = 7;

passive_mean_step = zeros(200, 2, passive_trials);
swing_mean_step = zeros(200, 2, swing_trials);

passive_std_step = zeros(200, 2, passive_trials);
swing_std_step = zeros(200, 2, swing_trials);

for trial = 1:passive_trials
    passive_mean_step(:, :, trial) = importdata(strcat(passive_data_path, 'Mean_step', num2str(trial), '.txt'));
    passive_std_step(:, :, trial) = importdata(strcat(passive_data_path, 'Std_step', num2str(trial), '.txt'));
end

for trial = 2:swing_trials
    swing_mean_step(:, :, trial) = importdata(strcat(swing_data_path, 'Mean_step', num2str(trial), '.txt'));
    swing_std_step(:, :, trial) = importdata(strcat(swing_data_path, 'Std_step', num2str(trial), '.txt'));
end

joint_names = {'Hip joint', 'Knee joint'};

cons_trials = [2, 3];
            
figure()
phase_normal = linspace(0, 1, 200)';
for k = 1:2
    
    subplot(2, 1, k);
    %errorfill(phase_normal', passive_mean_step(:, k, 1)'*180/pi, passive_std_step(:, k, 1)'*180/pi, 'r-', 1);
    %hold on;
    errorfill(phase_normal', swing_mean_step(:, k, cons_trials(1))',...
                             swing_std_step(:, k, cons_trials(1))', 'r-', 1.0);
    hold on;
    errorfill(phase_normal', swing_mean_step(:, k, cons_trials(2))',...
                             swing_std_step(:, k, cons_trials(2))', 'b-', 0.5);
    hold on;  
%     plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
%     hold on
%     plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
    title(joint_names{k});
    
end
hold off
sgtitle('Joint Motion in Step (Constant Speed)');
legend('Normal Walking std', 'Normal Walking mean', 'Passive Walking std', ...
       'Passive Walking mean', 'Swing Control std', 'Swing Control mean')

pert_trials = [5, 7];

figure()
phase_normal = linspace(0, 1, 200)';
for k = 1:2
    subplot(2, 1, k);
%     errorfill(phase_normal', passive_mean_step(:, k, 2)'*180/pi, passive_std_step(:, k, 2)'*180/pi, 'r-', 1);
%     hold on;
    errorfill(phase_normal', swing_mean_step(:, k, pert_trials(2))',...
                             swing_std_step(:, k, pert_trials(2))', 'r-', 1.0);
    hold on;
    errorfill(phase_normal', swing_mean_step(:, k, pert_trials(1))',...
                             swing_std_step(:, k, pert_trials(1))', 'b-', 0.5);
    hold on;
   
%     plot([0, 0.6], [0, 0], 'k-', 'linewidth', 3);
%     hold on
%     plot([0.6, 1.0], [0, 0], 'Color', [0.75 0.75 0.75], 'linewidth', 3);
    title(joint_names{k});
end
hold off
sgtitle('Joint Motion  in Step (Perturbed Speed)');
legend('Normal Walking std','Normal Walking mean', 'Passive Walking std', ...
       'Passive Walking mean', 'Swing Control std', 'Swing Control mean')
