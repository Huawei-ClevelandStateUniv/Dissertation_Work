%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code generate the comparison plot of EMGs
% Trial 1: Passive walking
% Trial 2: Walking with right side swing control
% Trial 3: Walking with two side swing control
% Trial 4: Walking with right side swing control with perturbed speed
% Trial 5: Passive walking with perturbed speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clc
clear

passive_data_path = '..\Participant_01\Cortex\passive_walking\';
swing_data_path = '..\Participant_01\Cortex\swing_control\';

passive_trials = 3;
swing_trials = 5;

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

cons_trials = [1, 2];

p_value = importdata(strcat(swing_data_path, 'Processed_EMG\P_value_anova_test_',...
            num2str(cons_trials(1)), '_', num2str(cons_trials(2)) , '.txt'));
        
p_value_sign = p_value < 0.05;
            
figure()
phase_normal = linspace(0, 1, 500)';
for k = 1:6
    subplot(3, 2, k);
%     errorfill(phase_normal', passive_EMG_mean_step(:, k, 1)', passive_EMG_std_step(:, k, 1)', 'r-', 1);
%     hold on;
    errorfill(phase_normal', swing_EMG_mean_step(:, k, cons_trials(1))',...
                             swing_EMG_std_step(:, k, cons_trials(1))', 'r-', 1.0);
    hold on;
    errorfill(phase_normal', swing_EMG_mean_step(:, k, cons_trials(2))',...
                             swing_EMG_std_step(:, k, cons_trials(2))', 'b-', 0.6);
    hold on;
    
    max_signal1 = max(swing_EMG_mean_step(:, k, cons_trials(2))' ...
                   + swing_EMG_std_step(:, k, cons_trials(2))');
               
    max_signal2 = max(swing_EMG_mean_step(:, k, cons_trials(1))' ...
                   + swing_EMG_std_step(:, k, cons_trials(1))');
               
    max_signal = max(max_signal1, max_signal2);
               
    plot(phase_normal', p_value_sign(:, k)*max_signal*1.2, '.', 'Color', [0.9290, 0.6940, 0.1250])
    
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
sgtitle('Muscle Activation in Step (Constant Speed)');
% 'Normal Walking std','Normal Walking mean',
legend('Passive Walking std', ...
       'Passive Walking mean', 'Swing Control std', 'Swing Control mean')

pert_trials = [4, 5];
   
p_value = importdata(strcat(swing_data_path, 'Processed_EMG\P_value_anova_test_',...
            num2str(pert_trials(1)), '_', num2str(pert_trials(2)) , '.txt'));
        
p_value_sign = p_value < 0.05;

figure()
phase_normal = linspace(0, 1, 500)';
for k = 1:6
    subplot(3, 2, k);
%     errorfill(phase_normal', passive_EMG_mean_step(:, k, 2)', passive_EMG_std_step(:, k, 2)', 'r-', 1);
%     hold on;
    errorfill(phase_normal', swing_EMG_mean_step(:, k, pert_trials(2))',...
                             swing_EMG_std_step(:, k, pert_trials(2))', 'r-', 1.0);
    hold on;
    errorfill(phase_normal', swing_EMG_mean_step(:, k, pert_trials(1))',...
                             swing_EMG_std_step(:, k, pert_trials(1))', 'b-', 0.6);
    hold on;
    
    max_signal1 = max(swing_EMG_mean_step(:, k, pert_trials(2))' ...
                   + swing_EMG_std_step(:, k, pert_trials(2))');
               
    max_signal2 = max(swing_EMG_mean_step(:, k, pert_trials(1))' ...
                   + swing_EMG_std_step(:, k, pert_trials(1))');
               
    max_signal = max(max_signal1, max_signal2);
               
    plot(phase_normal', p_value_sign(:, k)*max_signal*1.2, '.', 'Color', [0.9290, 0.6940, 0.1250])
    
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
sgtitle('Muscle Activation in Step (Perturbed Speed)');
% 'Normal Walking std','Normal Walking mean',
legend('Passive Walking std', ...
       'Passive Walking mean', 'Swing Control std', 'Swing Control mean')
