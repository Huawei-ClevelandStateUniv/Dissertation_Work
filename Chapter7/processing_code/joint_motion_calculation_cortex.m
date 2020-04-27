clc
clear
close all

data_path = '..\Participant_02\Cortex\passive_walking\';

trial = 2;

%% Process Mocap data
mocap_data = importdata(strcat(data_path, 'Mocap000', ...
                        num2str(trial) ,'.txt'));

num_markers = 14;
mass = 65;

mocap_time = mocap_data.data(:, 1) - mocap_data.data(1, 1);

F2_mocap = mocap_data.data(:, 57:62);

% 1. Fill the gaps in marker data and generate the gap information.
[filled_marker_data, gap_info] = FillMarkerGap(mocap_data.data(:,...
                                               1:num_markers*3 + 2));

% 2. Calcuate joint angles
Rmarkers_data = [-filled_marker_data(:, 5), filled_marker_data(:, 4),...
                         -filled_marker_data(:, 29), filled_marker_data(:, 28),...
                         -filled_marker_data(:, 32), filled_marker_data(:, 31),...
                         -filled_marker_data(:, 35), filled_marker_data(:, 34),...
                         -filled_marker_data(:, 38), filled_marker_data(:, 37),...
                         -filled_marker_data(:, 44), filled_marker_data(:, 43)];       

F2_2d = [-F2_mocap(:, 3), F2_mocap(:, 2), F2_mocap(:, 4)];

options.freq = 8;
                                                            
[Rangles, ~, Rtorques, ~] = leg2d(mocap_time, Rmarkers_data,...
                                              F2_2d/mass, options);
                                          
Rangles(:, 3) = Rangles(:, 3) + pi/2;
%% Compare joint angles between motion capture data and Indego simulink data

figure(1)
subplot(3,2,1)
plot(mocap_time, Rangles(:, 1)*180/pi, '-')
ylabel('Hip Joint (deg.)')
title('Motion')

subplot(3,2,2)
plot(mocap_time, Rtorques(:, 1)*mass, '-')
ylabel('Nm')
title('Moment')

subplot(3,2,3)
plot(mocap_time, Rangles(:, 2)*180/pi, '-')
ylabel('Knee Joint (deg.)')

subplot(3,2,4)
plot(mocap_time, Rtorques(:, 2)*mass, '-')
ylabel('Nm')

subplot(3,2,5)
plot(mocap_time, Rangles(:, 3)*180/pi, '-')
ylabel('Ankle Joint (deg.)')
xlabel('Time (second)')

subplot(3,2,6)
plot(mocap_time, Rtorques(:, 3)*mass, '-')
ylabel('Nm')
xlabel('Time (second)')

%% Save generated motion data

fid = fopen(strcat(data_path, 'motion_right_trial0', num2str(trial),'.txt'), 'w');
for iy = 1:length(Rangles(:, 1))
    fprintf(fid,'%d ', Rangles(iy, :));  % then the data
    fprintf(fid, '\n');
end
fclose(fid);

fid = fopen(strcat(data_path, 'moment_right_trial0', num2str(trial),'.txt'), 'w');
for iy = 1:length(Rtorques(:, 1))
    fprintf(fid,'%d ', Rtorques(iy, :));  % then the data
    fprintf(fid, '\n');
end
fclose(fid);

%% Detect Heel Strike and Toe Off

[b, a] = butter(2, (8/(100/2)));
Fy2 = filtfilt(b, a, F2_mocap(:, 2));

figure()
plot(Fy2)
legend('Fy2 (N)')
print('Select the threshold of detecting heel strike of right leg...')
[~, Y2_threshold] = ginput(1);

analog_time = 0:0.01:(length(Fy2)-1)*0.01;

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

right_step_index = zeros(min_steps - 1, 3);

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

%% Save step detection data
fid = fopen(strcat(data_path, 'motion_hs_to_index_right_trial0', num2str(trial),'.txt'), 'w');
for iy = 1:length(right_step_index(:, 1))
    fprintf(fid,'%d ', right_step_index(iy, :));  % then the data
    fprintf(fid, '\n');
end
fclose(fid);

%% Calculate averaged data

step_normal = zeros(200, 2, length(right_step_index(:, 1)));
moment_normal = zeros(200, 2, length(right_step_index(:, 1)));

for s = 1:length(right_step_index(:, 1))

    step = Rangles(right_step_index(s, 1):right_step_index(s, 3) ,1:2);
    moment = Rtorques(right_step_index(s, 1):right_step_index(s, 3) ,1:2);
    time_indego = (0:0.005:(right_step_index(s, 3) - right_step_index(s, 1))*0.005)';
    time_normal = linspace(0, (right_step_index(s, 3) - right_step_index(s, 1))*0.005, 200)';
    step_normal(:, :, s) = interp1(time_indego, step, time_normal, 'linear', 'extrap');  
    moment_normal(:, :, s) = interp1(time_indego, moment, time_normal, 'linear', 'extrap'); 

end

    mean_step = mean(step_normal, 3);
    std_step = std(step_normal, 0, 3);
    
    mean_moment = mean(moment_normal, 3);
    std_moment = std(moment_normal, 0, 3);

    fid = fopen(strcat(data_path, 'Mean_step', num2str(trial), '.txt'), 'w');

    for iy = 1:length(mean_step(:, 1))
        fprintf(fid,'%d ', mean_step(iy, :));  % then the data
        fprintf(fid, '\n');
    end
    fclose(fid);

    fid = fopen(strcat(data_path, 'Std_step', num2str(trial), '.txt'), 'w');

    for iy = 1:length(std_step(:, 1))
        fprintf(fid,'%d ', std_step(iy, :));  % then the data
        fprintf(fid, '\n');
    end
    fclose(fid);
    
    
    fid = fopen(strcat(data_path, 'Mean_moment', num2str(trial), '.txt'), 'w');

    for iy = 1:length(mean_moment(:, 1))
        fprintf(fid,'%d ', mean_moment(iy, :));  % then the data
        fprintf(fid, '\n');
    end
    fclose(fid);

    fid = fopen(strcat(data_path, 'Std_moment', num2str(trial), '.txt'), 'w');

    for iy = 1:length(std_moment(:, 1))
        fprintf(fid,'%d ', std_moment(iy, :));  % then the data
        fprintf(fid, '\n');
    end
    fclose(fid);

