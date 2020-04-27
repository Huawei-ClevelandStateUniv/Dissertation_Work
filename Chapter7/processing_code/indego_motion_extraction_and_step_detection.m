clc
clear
close all

data_path_cortex = '..\Participant_02\Cortex\swing_control\';
data_path_indego = '..\Participant_02\Indego\swing_control\';

trial = 7;

indego = load(strcat(data_path_indego, 'indego_trial_0', ...
                        num2str(trial) ,'.mat'));
         
mocap = importdata(strcat(data_path_cortex, 'Mocap000', ...
                    num2str(trial) ,'.txt'));
        
[b, a] = butter(2, 8/(100/2));

Fy1 = filtfilt(b, a, mocap.data(:, 7));
Fy2 = filtfilt(b, a, mocap.data(:, 16));

if trial == 2
    t_st = 830*2;
    t_ed = t_st + 15000*2;
end

if trial == 3
    t_st = 600*2;
    t_ed = t_st + 15000*2;
end

if trial == 4
    t_st = 400*2;
    t_ed = t_st + 15000*2;
end

if trial == 5
    t_st = 900*2;
    t_ed = t_st + 15000*2;
end

if trial == 6
    t_st = 460*2;
    t_ed = t_st + 15000*2;
end

if trial == 7
    t_st = 500*2;
    t_ed = t_st + 15000*2;
end

simulink_time = indego.leftFusedHipKnee.time(t_st:t_ed) ...
                    - indego.leftFusedHipKnee.time(t_st);

left_joint = [indego.leftFusedHipKnee.signals(1).values(t_st:t_ed),...
              indego.leftFusedHipKnee.signals(2).values(t_st:t_ed)];
          
right_joint = [indego.rightFusedHipKnee.signals(1).values(t_st:t_ed),...
              indego.rightFusedHipKnee.signals(2).values(t_st:t_ed)];
          
left_joint_absolute = [indego.leftThighAngleVelocity.signals(1).values(t_st:t_ed),...
              indego.leftFusedHipKnee.signals(2).values(t_st:t_ed)];
          
right_joint_absolute = [indego.rightThighAngleVelocity.signals(1).values(t_st:t_ed),...
              indego.rightFusedHipKnee.signals(2).values(t_st:t_ed)];

dflow_time = linspace(0, (length(Fy1)-1)*0.01, length(Fy1));

F1y_simulink = interp1(dflow_time, Fy1, simulink_time, 'linear', 'extrap');
F2y_simulink = interp1(dflow_time, Fy2, simulink_time, 'linear', 'extrap');

figure()
subplot(2,1,1)
title('Left Leg')
plot(simulink_time, left_joint)
hold on
plot(simulink_time, left_joint_absolute, '--')
hold on
plot(simulink_time, F1y_simulink*0.1)
hold off
legend('relative angles', 'absolute angles')

subplot(2,1,2)
title('Right Leg')
plot(simulink_time, right_joint)
hold on
plot(simulink_time, right_joint_absolute, '--')
hold on
plot(simulink_time, F2y_simulink*0.1)
hold off
legend('relative angles', 'absolute angles')

%% Detect Heel Strike and Toe Off
sign_left = zeros(length(F1y_simulink), 1);
sign_right = zeros(length(F2y_simulink), 1);

figure()
plot(F2y_simulink)
legend('Fy2 (N)')
print('Select the threshold of detecting heel strike of right leg...')
[~, Y2_threshold] = ginput(1);

for k = 1:length(simulink_time)
    if F2y_simulink(k) > Y2_threshold(1)
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

%% Remove unnormal gaits

% copy previous phase detector

min_steps = min(length(hs_right), length(to_right));

hs_right1 = hs_right(1:min_steps-1);
to_right1 = to_right(1:min_steps-1);
hs_right1_end = hs_right(2:min_steps);

y = 50;
close all;

while y > 10
    
    right_step_index = [hs_right1(1:min_steps-1), ...
                        to_right1(1:min_steps-1), ...
                        hs_right1_end(1:min_steps-1)];

    
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
    hs_right1(round(x)) = [];
    min_steps = min_steps - 1;
elseif y < 2*y_max/3
    to_right1(round(x)) = [];
    min_steps = min_steps - 1;
elseif y < 3*y_max/3
    hs_right1_end(round(x)) = [];
    min_steps = min_steps - 1;
else 
    hs_right1(round(x)) = [];
    to_right1(round(x)) = [];
    hs_right1_end(round(x)) = [];
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

fid = fopen(strcat(data_path_indego, 'motion_hs_to_index_right_trial0', num2str(trial),'.txt'), 'w');
for iy = 1:length(right_step_index(:, 1))
    fprintf(fid,'%d ', right_step_index(iy, :));  % then the data
    fprintf(fid, '\n');
end
fclose(fid);
