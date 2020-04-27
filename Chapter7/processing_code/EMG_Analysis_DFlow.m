clc;
clear;
close all;
trial = 1;

%% load analog data

analog_data = importdata(strcat('..\Participant_02\Cortex\passive_walking\Mocap000', ...
                        num2str(trial) ,'_Motion Analysis_analog.txt'));

analog_time = (0:0.001:(length(analog_data.data(:, 1))-1)*0.001)';
analog_1_6 = analog_data.data(:, 3:8);
analog_7_12 = analog_data.data(:, 9:14);
EMG = analog_data.data(:, [17, 21, 25, 29, 33, 45] + 2  );

[b1, a1] = butter(2, 25/(1000/2), 'high');
[b2, a2] = butter(2, 6/(1000/2), 'low');

EMG1 = filtfilt(b1, a1, EMG); % High pass filter, 25Hz
EMG2 = abs(EMG1); % Rectify
EMG3 = filtfilt(b2, a2, EMG2); % Low pass filter, 6Hz

[F1, F2] = Regain_GRF(analog_1_6, analog_7_12); % Generate GRF based on analog signals

%% Detect Heel Strike and Toe Off
sign_left = zeros(length(analog_time), 1);
sign_right = zeros(length(analog_time), 1);

for k = 1:length(analog_time)
    if F1(k, 2) > 150
        sign_left(k) = 1;
    end
    
    if F2(k, 2) > 150
        sign_right(k) = 1;
    end
end

hs_left = find(diff(sign_left) == 1);
to_left = find(diff(sign_left) == -1);

hs_right = find(diff(sign_right) == 1);
to_right = find(diff(sign_right) == -1);
% 
% figure()
% plot(diff(hs_left),'.')
% hold on
% plot(diff(to_left), '.')
% hold off
% legend('Heel Strike', 'Toe Off')
% ylabel('Duration (frames)')
% xlabel('Dectect Numbers')
% title('Left HS & TO Durations')
% [~,left_s] = ginput(1);
left_s = 857.98;

hs_left_remove = find(diff(hs_left) < left_s);
to_left_remove = find(diff(to_left) < left_s);

hs_left(hs_left_remove) = [];
to_left(to_left_remove+1) = [];

figure()
plot(diff(hs_left),'.')
hold on
plot(diff(to_left), '.')
hold off
legend('Heel Strike', 'Toe Off')
ylabel('Duration (frames)')
xlabel('Dectect Numbers')
title('Processed Left HS & TO Durations')


% figure()
% plot(diff(hs_right),'.')
% hold on
% plot(diff(to_right), '.')
% hold off
% legend('Heel Strike', 'Toe Off')
% ylabel('Duration (frames)')
% xlabel('Dectect Numbers')
% title('Right HS & TO Durations')
% [~,right_s] = ginput(1);
right_s = 803.502;

hs_right_remove = find(diff(hs_right) < right_s);
to_right_remove = find(diff(to_right) < right_s);

hs_right(hs_right_remove) = [];
to_right(to_right_remove+1) = [];

figure()
plot(diff(hs_right),'.')
hold on
plot(diff(to_right), '.')
hold off
legend('Heel Strike', 'Toe Off')
ylabel('Duration (frames)')
xlabel('Dectect Numbers')
title('Processed Right HS & TO Durations')

%% Generate step stance and swing index period (right leg where EMG was placed)

minimum_steps_right = min([length(hs_right),length(to_right)]);

right_step_index = zeros(minimum_steps_right - 1, 2);
right_swing_index = zeros(minimum_steps_right - 1, 2);

for s = 1:minimum_steps_right - 1
    right_step_index(s, 1) = hs_right(s);
    right_step_index(s, 2) = hs_right(s+1);
end

for s = 1:minimum_steps_right - 1
    right_swing_index(s, 1) = to_right(s);
    right_swing_index(s, 2) = hs_right(s);
end

% figure()
% plot(right_step_index(:, 2) - right_step_index(:, 1))
% title('Step period of right leg')
% [~, y_left] = ginput(1);

% figure()
% plot(right_swing_index(:, 2) - right_swing_index(:, 1))
% title('Swing period of right leg')
% [~, y_right] = ginput(1);
y_right = 531.81;

%remove_swing_l_index = find(left_swing_index(:, 2) - left_swing_index(:, 1) < y_left);
remove_swing_r_index = find(right_swing_index(:, 2) - right_swing_index(:, 1) < y_right);

%right_step_index(remove_swing_l_index, :) = [];
%right_swing_index(remove_swing_r_index, :) = [];

% figure()
% plot(left_swing_index(:, 2) - left_swing_index(:, 1))
% title('Swing period of left leg')

figure()
plot(right_swing_index(:, 2) - right_swing_index(:, 1))
title('Swing period of right leg')

figure()
plot(right_step_index(:, 2) - right_step_index(:, 1))
title('Step period of right leg')

%% Average EMG in the swing phase
EMG_averg_swing = zeros(250, 6, length(right_swing_index(:, 2)));
EMG_averg_step = zeros(500, 6, length(right_step_index(:, 2)));
for s = 5:length(right_swing_index(:, 2))-5
    
    EMG_swing = EMG3(right_swing_index(s, 1)+48:right_swing_index(s, 2)+48 ,:);  %EMG delay 48 ms
    time_analog = (0:0.001:(right_swing_index(s, 2) - right_swing_index(s, 1))*0.001)';
    time_normal = linspace(0, (right_swing_index(s, 2) - right_swing_index(s, 1))*0.001, 250)';
    EMG_averg_swing(:, :, s) = interp1(time_analog, EMG_swing, time_normal, 'linear', 'extrap');  
    
    EMG_step = EMG3(right_step_index(s, 1)+48:right_step_index(s, 2)+48 ,:);  %EMG delay 48 ms
    time_analog = (0:0.001:(right_step_index(s, 2) - right_step_index(s, 1))*0.001)';
    time_normal = linspace(0, (right_step_index(s, 2) - right_step_index(s, 1))*0.001, 500)';
    EMG_averg_step(:, :, s) = interp1(time_analog, EMG_step, time_normal, 'linear', 'extrap');  
    
end

EMG_mean_swing = mean(EMG_averg_swing, 3);
EMG_std_swing = std(EMG_averg_swing, 0, 3);

EMG_mean_step = mean(EMG_averg_step, 3);
EMG_std_step = std(EMG_averg_step, 0, 3);

muscle_names = {'Rectus Femoris', 'Vastus lateral', 'Biceps Femoris',...
    'Semitendinous', 'Lateral Gastrocnemius', 'Tibialis'};

figure()
title('Muscle Activation in Swing')
phase_normal = linspace(0.5, 1, 250)';
for k = 1:6
    subplot(3, 2, k);
    errorfill(phase_normal', EMG_mean_swing(:, k)', EMG_std_swing(:, k)', 'r-');
    hold on;
    plot([0.5, 1], [0, 0], 'k--');
    title(muscle_names{k});
end
hold off


figure()
title('Muscle Activation in Step')
phase_normal = linspace(0, 1, 500)';
for k = 1:6
    subplot(3, 2, k);
    errorfill(phase_normal', EMG_mean_step(:, k)', EMG_std_step(:, k)', 'r-');
    hold on;
    plot([0, 1], [0, 0], 'k--');
    title(muscle_names{k});
end
hold off





%% Subfunction
function [F1, F2] = Regain_GRF(analog_1_6, analog_7_12)

    CalibrationMatrix_l = [3.211, -4.296, 9.517, -506.156, -503.077, -12.798;...
                         -488.7, -489.177, -490.137, 12.72, -12.405, 6.997;...
                         -2.439, -22.137, -12.713, -5.808, 0.306, -529.752;...
                         51.205, -296.238, 301.239, 0, 0, 54.628;...
                         0, 0, 0, -252.497, 357.263, -9.06;...
                         249.758, 24.2, 20.303, -52.477, -52.08, 0;];
                     
                     
    CalibrationMatrix_r = [10.127, -0.128, 7.117, 502.928, 498.685, -10.051;...
                         -485.724, -486.699, -488.278, 16.96, 24.012, 1.989;...
                         -3.781, -5.618, -17.688, -4.968, 0.7, -516.168;...
                         297.882, -297.409, 45.867, 0, 0, 52.865;...
                         0, 0, 0, -357.685, 248.453, 8.76;...
                         -30.947, -27.516, -254.202, 52.141, 51.636, 0;];
                     
    F1 = -analog_1_6*CalibrationMatrix_l';
    F2 = -analog_7_12*CalibrationMatrix_r';
    
end
