clc
clear
close all

% load normal walking data
normal_trial = 1;
motion_normal = importdata(strcat('../Participant_02/Cortex/passive_walking/motion_right_trial0'...
                            , num2str(normal_trial), '.txt'))*180/pi;
index_normal = importdata(strcat('../Participant_02/Cortex/passive_walking/motion_hs_to_index_right_trial0'...
                            , num2str(normal_trial), '.txt'));
                        
cortex_time = linspace(0, (length(motion_normal(:, 1))-1)*0.01, length(motion_normal(:, 1)))';

% load passive walking data
passive_trial = 1;
if passive_trial == 1
    t_st = 5010*2;
    t_ed = t_st + 21500*2;
elseif passive_trial == 2
    t_st = 9730*2;
    t_ed = t_st + 15000*2;
end

% Process Simulink Recorded data
motion_passive = load(strcat('../Participant_02/Indego/passive_walking/indego_trial_0'...
                        , num2str(passive_trial), '.mat'));                  
index_passive = importdata(strcat('../Participant_02/Indego/passive_walking/left_to_right_phase_index'...
                        , num2str(passive_trial), '.txt')).data;

simulink_time = motion_passive.leftFusedHipKnee.time(t_st:t_ed) ...
              - motion_passive.leftFusedHipKnee.time(t_st);
          
left_joint_absolute = [motion_passive.leftThighAngleVelocity.signals(1).values(t_st:t_ed),...
              motion_passive.leftFusedHipKnee.signals(2).values(t_st:t_ed)];
          
right_joint_absolute = [motion_passive.rightThighAngleVelocity.signals(1).values(t_st:t_ed),...
              motion_passive.rightFusedHipKnee.signals(2).values(t_st:t_ed)];

%%
figure()
subplot(2,2,1)
title('Normal Walking')
plot(cortex_time, motion_normal(:, 1), '-')
hold on
plot(cortex_time, motion_normal(:, 2), '--')
hold off
legend('Hip Joint', 'Knee Joint')

subplot(2,2,2)
title('Passive Walking')
plot(simulink_time, left_joint_absolute(:, 1), '-')
hold on
plot(simulink_time, left_joint_absolute(:, 2), '--')
hold off
legend('Hip Joint', 'Knee Joint')

subplot(2,2,3)
title('Phase')
for i = 1:length(index_normal(:, 1))
plot([index_normal(i, 1),index_normal(i, 2)], [0, 0], 'r-', 'linewidth', 2.5);
hold on
plot([index_normal(i, 2),index_normal(i, 3)], [1, 1], 'g--', 'linewidth', 2.5);
hold on
end
hold off

subplot(2,2,4)
title('Phase')
for i = 1:length(index_passive(:, 1))
plot([index_passive(i, 1),index_passive(i, 2)], [0, 0], 'r-', 'linewidth', 2.5);
hold on
plot([index_passive(i, 2),index_passive(i, 3)], [1, 1], 'g--', 'linewidth', 2.5);
hold on
plot([index_passive(i, 3),index_passive(i, 4)], [2, 2], 'k.-', 'linewidth', 2.5);
hold on
end
hold off

%% calculate pelvis position and velocity based on joint angle and angular
% velocity

ls = 0.35;
lt = 0.41;

[b1, a1] = butter(2, 6/(100/2));
motion_normal = filtfilt(b1, a1, motion_normal)*pi/180;

[b2, a2] = butter(2, 6/(200/2));
motion_passive = filtfilt(b2, a2, left_joint_absolute)*pi/180;

min_steps_passive = length(index_passive(:, 1));
min_steps_normal = length(index_normal(:, 1));

pelvis_normal = {};
pelvis_normal_v_diff = {};

pelvis_passive = {};
pelvis_passive_v_diff = {};


for s = 1:min_steps_normal - 1
    
    pelvis_normal{end+1} = -ls*sin(motion_normal(index_normal(s, 1):...
                                                index_normal(s, 2), 1) ...
                        - motion_normal(index_normal(s, 1):...
                                         index_normal(s, 2), 2)) ...
                        - lt*sin(motion_normal(index_normal(s, 1):...
                                                index_normal(s, 2), 1));
                    
    pelvis_normal_v_diff{end+1} = diff(pelvis_normal{s})/0.01;
                    
end

for s = 1:min_steps_passive - 1
    
    pelvis_passive{end+1} = -ls*sin(motion_passive(index_passive(s, 1):...
                                                index_passive(s, 4), 1) ...
                        - motion_passive(index_passive(s, 1):...
                                         index_passive(s, 4), 2)) ...
                        - lt*sin(motion_passive(index_passive(s, 1):...
                                                index_passive(s, 4), 1));
                    
    pelvis_passive_v_diff{end+1} = diff(pelvis_passive{s})/0.005;
                    
end

%% Plot pelvis postion and velocity

figure()
subplot(2,2,1)
for i = 1:min_steps_normal-1
plot(pelvis_normal{i})
hold on
end
ylabel('Pelvis Position (m)')
title('Normal Walking')
axis([0, 100, -0.4, 0.4])
subplot(2,2,2)
for i = 1:min_steps_passive-1
plot(pelvis_passive{i})
hold on
end
title('Walking with Passive Indego')
axis([0, 200, -0.4, 0.4])
subplot(2,2,3)
for i = 1:min_steps_normal-1
plot(pelvis_normal_v_diff{i})
hold on
end
ylabel('Pelvis Velocity (m/s)')
xlabel('Data Nodes (100Hz)')
axis([0, 100, -1, 2])
subplot(2,2,4)
for i = 1:min_steps_passive-1
plot(pelvis_passive_v_diff{i})
hold on
end
xlabel('Data Nodes (200Hz)')
axis([0, 200, -1, 2])
%sgtitle('Pelvis position and velocity relative to stance foot')
