clc;
clear;
close all;

data_path = '..\Participant_02\Cortex\swing_control\';

trial = 7;

%% load analog data

analog_data = importdata(strcat(data_path, 'Mocap000', ...
                        num2str(trial) ,'_Motion Analysis_analog.txt'));

analog_time = (0:0.001:(length(analog_data.data(:, 1))-1)*0.001)';
[b2, a2] = butter(2, 6/(1000/2), 'low');
analog_1_6 = analog_data.data(:, 3:8);
analog_7_12 = analog_data.data(:, 9:14);
[F1, F2] = Regain_GRF(analog_1_6, analog_7_12); % Generate GRF based on analog signals

Fy1 = filtfilt(b2, a2, F1(:, 2));
Fy2 = filtfilt(b2, a2, F2(:, 2));

figure()
plot(Fy2)
legend('Fy2 (N)')
print('Select the threshold of detecting heel strike of right leg...')
[~, Y2_threshold] = ginput(1);

%% Detect Heel Strike and Toe Off
sign_left = zeros(length(analog_time), 1);
sign_right = zeros(length(analog_time), 1);

for k = 1:length(analog_time)
    
    if Fy2(k) > Y2_threshold
        sign_right(k) = 1;
    end
end

hs_right = find(diff(sign_right) == 1);
to_right = find(diff(sign_right) == -1);

figure()
plot(diff(hs_right),'.')
hold on
plot(diff(to_right), '.')
hold off
legend('Heel Strike', 'Toe Off')
ylabel('Duration (frames)')
xlabel('Dectect Numbers')
title('Right HS & TO Durations')
[~,right_s] = ginput(1);

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

min_steps = min([length(hs_right),length(to_right)]);

hs_right_end = hs_right(2:min_steps);

y = 50;
close all;

while y > 10
    
    right_step_index = [hs_right(1:min_steps-1), ...
                        to_right(1:min_steps-1), ...
                        hs_right_end(1:min_steps-1)];

    
    y_max = max([max(right_step_index(:, 3) - right_step_index(:, 2)), ...
             max(right_step_index(:, 2) - right_step_index(:, 1))]);

    figure()
    plot(right_step_index(:, 2) - right_step_index(:, 1), '.')
    hold on
    plot(right_step_index(:, 3) - right_step_index(:, 2), '.')
    hold on
    plot([1, min_steps], [0, 0], '--')
    hold on
    plot([1, min_steps], [1*y_max/3, 1*y_max/3], '--')
    hold on
    plot([1, min_steps], [2*y_max/3, 2*y_max/3], '--')
    hold on
    plot([1, min_steps], [3*y_max/3, 3*y_max/3], '--')
    hold off
    title ('Cleaned step phase frames')
    legend({'frame numbers from heel strike to toe off', ...
           'frame numbers from toe off to heel strike'}, ...
           'Location', 'northwest', 'NumColumns', 2)

[x, y] = ginput(1);

if y > 1 && y < 1*y_max/3
    hs_right(round(x)) = [];
    min_steps = min_steps - 1;
elseif y < 2*y_max/3
    to_right(round(x)) = [];
    min_steps = min_steps - 1;
elseif y < 3*y_max/3
    hs_right_end(round(x)) = [];
    min_steps = min_steps - 1;
else 
    hs_right(round(x)) = [];
    to_right(round(x)) = [];
    hs_right_end(round(x)) = [];
    min_steps = min_steps - 1;
end

end
close all


figure()
plot(right_step_index(:, 3) - right_step_index(:, 2), '.')
hold on
plot(right_step_index(:, 2) - right_step_index(:, 1), '.')
hold off
legend('Swing period of right leg', 'Stance period of right leg')

%% Save the hs and to index data

fid = fopen(strcat(data_path, 'EMG_hs_to_index_right_trial0', num2str(trial),'.txt'), 'w');
for iy = 1:length(right_step_index(:, 1))
    fprintf(fid,'%d ', right_step_index(iy, :));  % then the data
    fprintf(fid, '\n');
end
fclose(fid);
