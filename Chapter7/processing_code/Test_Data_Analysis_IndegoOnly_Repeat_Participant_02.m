clc
clear
close all

trial = 1;

% if trial == 1
%     t_st = 5010*2;
%     t_ed = t_st + 21500*2;
% elseif trial == 2
%     t_st = 3730*2;
%     t_ed = t_st + 18000*2;
% end

% Process Simulink Recorded data
left_joint = importdata(strcat('..\Participant_02\Forward_simulation\left_joint_angles000'...
                        , num2str(trial) ,'.txt'));
right_joint = importdata(strcat('..\Participant_02\Forward_simulation\right_joint_angles000'...
                        , num2str(trial) ,'.txt'));
vertical_force = importdata(strcat('..\Participant_02\Forward_simulation\vertical_force000'...
                        , num2str(trial) ,'.txt'));

figure()
subplot(2,2,1)
title('Left Leg')
plot(left_joint(:, 1), left_joint(:, 2:3))
legend('hip angles', 'knee angles')

subplot(2,2,2)
title('Right Leg')
plot(right_joint(:, 1), right_joint(:, 2:3))
legend('hip angles', 'knee angles')

subplot(2,2,3)
title('Left GRF')
plot(vertical_force(:, 1), vertical_force(:, 2))

subplot(2,2,4)
title('Right GRF')
plot(vertical_force(:, 1), vertical_force(:, 3))

%% calculate pelvis position and velocity based on joint angle and angular
% velocity

left_to_right_index_dataset = importdata(strcat('..\Participant_02\Indego\passive_walking\left_to_right_phase_index',...
                      num2str(trial), '.txt'));
                  
left_to_right_index =  left_to_right_index_dataset.data;     
global min_steps 
min_steps = length(left_to_right_index(:, 1));

ls = 0.35;
lt = 0.41;

[b, a] = butter(2, 8/(200/2));
filted_left_joint = filtfilt(b, a, left_joint(:, 2:3))*pi/180; % filter and transfer to rad
 
left_joint_vel = [diff(filted_left_joint(:, 1))/0.005,...
    diff(filted_left_joint(:, 2))/0.005];

pelvis_left = {};
pelvis_left_y = {};
pelvis_left_v = {};

figure()
subplot(2,1,1)
for s = 1:min_steps -1
    
    pelvis_left{end+1} = -ls*sin(filted_left_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1) ...
                        - filted_left_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 2)) ...
                        - lt*sin(filted_left_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1));
                    
    pelvis_left_y{end+1} = ls*cos(filted_left_joint(left_to_right_index(s, 2):...
                    left_to_right_index(s, 3), 1) ...
                    - filted_left_joint(left_to_right_index(s, 2):...
                    left_to_right_index(s, 3), 2)) ...
                    + lt*cos(filted_left_joint(left_to_right_index(s, 2):...
                    left_to_right_index(s, 3), 1));
                    
    plot(pelvis_left{end})
    hold on
              
end
title('Pelvis position relative to stance ankle')
ylabel('m')
hold off

subplot(2,1,2)
for s = 1:min_steps -1
                    
    pelvis_left_v{end+1} = -ls*cos(filted_left_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1) ...
                        - filted_left_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 2))...
                        .*(left_joint_vel(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1) ...
                        - left_joint_vel(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 2)) ...
                        - lt*cos(filted_left_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1))...
                        .*(left_joint_vel(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1));
                    
                    
    plot(pelvis_left_v{end})
    hold on
    
end

title('Pelvis velocity relative to stance ankle')
ylabel('m/s')
xlabel('number of frames (200Hz)')
hold off

%% Calculate swing trajectory

%filted_right_joint = right_joint*pi/180; % filter and transfer to rad

filted_right_joint = right_joint(:, 2:3)*pi/180; % filter and transfer to rad
 
right_joint_vel = [diff(filted_right_joint(:, 1))/0.005,...
    diff(filted_right_joint(:, 2))/0.005];

pelvis_right = {};
pelvis_right_y = {};
global ankle_right ankle_right_y

ankle_right = {};
ankle_right_y = {};

figure()
subplot(2,1,1)
for s = 1:min_steps -1
    
    pelvis_right{end+1} = ls*sin(filted_right_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1) ...
                        - filted_right_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 2)) ...
                        + lt*sin(filted_right_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1));
                    
    pelvis_right_y{end+1} = -ls*cos(filted_right_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1) ...
                        - filted_right_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 2)) ...
                        - lt*cos(filted_right_joint(left_to_right_index(s, 2):...
                        left_to_right_index(s, 3), 1));
                    
    plot(pelvis_right{end})
    hold on
              
end
title('swing ankle position relative to pelvis position')
ylabel('m')
hold off

subplot(2,1,2)

for s = 1:min_steps -1
                    
    ankle_right{s} = pelvis_right{s} + pelvis_left{s};
    ankle_right_y{s} = pelvis_right_y{s} + pelvis_left_y{s};
                                 
    plot(ankle_right{s})
    hold on
    plot(ankle_right_y{s})
    hold on
    
end

title('swing ankle position relative to stance ankle')
ylabel('m')
xlabel('number of frames (200Hz)')
hold off

%% calculate polynimal functions of the swing x path

X0 = [-0.2; 0.3];

res = fmincon(@nploynimal, X0);

x_fit = {};
rms = [];

for s = 1 : min_steps - 1

    traj = ankle_right{s};

    T = length(traj)*0.005 - 0.005;

    t = (0:0.005:T)';

    x_fit{s} = traj(1) + res(1)*(traj(end) - traj(1)).*(t./T)...
                    + res(2)*(traj(end) - traj(1)).*(t./T).^2 ...
                    + (1-  res(1) - res(2))*(traj(end) - traj(1)).*(t./T).^3;

    rms(s) = sum((traj - x_fit{s}).^2);

end

[sd,r]=sort(rms,'descend');

best_ind = r(end);
worst_ind = r(1);
median_ind = r(round(length(r)/2));


figure()
subplot(1, 3, 1)
title('Best Fit')
plot((0:0.005:length(ankle_right{best_ind})*0.005 - 0.005)', ankle_right{best_ind}, 'r-')
hold on
plot((0:0.005:length(ankle_right{best_ind})*0.005 - 0.005)', x_fit{best_ind}, 'b--')
hold off

subplot(1, 3, 2)
title('Median Fit')
plot((0:0.005:length(ankle_right{median_ind})*0.005 - 0.005)', ankle_right{median_ind}, 'r-')
hold on
plot((0:0.005:length(ankle_right{median_ind})*0.005 - 0.005)', x_fit{median_ind}, 'b--')
hold off

subplot(1, 3, 3)
title('Worst Fit')
plot((0:0.005:length(ankle_right{worst_ind})*0.005 - 0.005)', ankle_right{worst_ind}, 'r-')
hold on
plot((0:0.005:length(ankle_right{worst_ind})*0.005 - 0.005)', x_fit{worst_ind}, 'b--')
hold off

figure()
subplot(2,1,1)
for s = 1: min_steps - 1
plot(ankle_right{s})
hold on
end
hold off
ylabel('Swing Motion X (m)')

subplot(2,1,2)
for s = 1: min_steps - 1
plot(x_fit{s}, '-.')
hold on
end
hold off
xlabel('Nodes (200Hz)')
ylabel('Polynomial Fit X (m)')
%% calculate polynimal functions of the swing y path

X0 = [-0.2, 0.3, 0.3, 0.5];

res_y = fmincon(@nploynimal_y, X0);

y_fit = {};
rms = [];

for s = 1 : min_steps - 1

    traj = ankle_right_y{s};

    T = length(traj)*0.005 - 0.005;

    t = (0:0.005:T)';

    y_fit{s} = traj(1) + res_y(1)*(traj(end) - traj(1)).*(t./T)...
                    + res_y(2)*(traj(end) - traj(1)).*(t./T).^2 ...
                    + res_y(3)*(traj(end) - traj(1)).*(t./T).^3 ...
                    + res_y(4)*(traj(end) - traj(1)).*(t./T).^4 ...
                    + (1-  res_y(1) - res_y(2) - res_y(3) - res_y(4)) ...
                    *(traj(end) - traj(1)).*(t./T).^5;

    rms(s) = sum((traj - y_fit{s}).^2);

end

[sd,r]=sort(rms,'descend');

best_ind = r(end);
worst_ind = r(1);
median_ind = r(round(length(r)/2));


figure()
subplot(1, 3, 1)
title('Best Fit')
plot((0:0.005:length(ankle_right_y{best_ind})*0.005 - 0.005)', ankle_right_y{best_ind}, 'r-')
hold on
plot((0:0.005:length(ankle_right_y{best_ind})*0.005 - 0.005)', y_fit{best_ind}, 'b--')
hold off

subplot(1, 3, 2)
title('Median Fit')
plot((0:0.005:length(ankle_right_y{median_ind})*0.005 - 0.005)', ankle_right_y{median_ind}, 'r-')
hold on
plot((0:0.005:length(ankle_right_y{median_ind})*0.005 - 0.005)', y_fit{median_ind}, 'b--')
hold off

subplot(1, 3, 3)
title('Worst Fit')
plot((0:0.005:length(ankle_right_y{worst_ind})*0.005 - 0.005)', ankle_right_y{worst_ind}, 'r-')
hold on
plot((0:0.005:length(ankle_right_y{worst_ind})*0.005 - 0.005)', y_fit{worst_ind}, 'b--')
hold off

figure()
subplot(2,1,1)
for s = 1: min_steps - 1
plot(ankle_right_y{s})
hold on
end
hold off
ylabel('Swing Motion Y (m)')
axis([0, 150, -0.05, 0.2])

subplot(2,1,2)
title('polynomial data')
for s = 1: min_steps - 1
plot(y_fit{s}, '-.')
hold on
end
hold off
xlabel('Nodes (200Hz)')
ylabel('Polynomial Fit Y (m)')
axis([0, 150, -0.05, 0.2])

%% Estimate step strategy based on joint angles

p_pos = 1.2;
p_vel = 1.2;
G = 9.8;
omega = sqrt(G/(ls+lt));

x_est = pelvis_left;
x_swing = pelvis_left;

figure()

plot([-1, 0], [-1, -1])
hold on
plot(-1, -1, 'k*')
hold on
for s = 1:length(pelvis_left)
    for f = 1:length(pelvis_left{s})
        
        T = length(pelvis_left{s})*0.005;
        
        x_ins = p_pos*pelvis_left{s}(f) + p_vel*pelvis_left_v{s}(f)/omega;
        x_est{s}(f) = x_ins*exp(omega*(T - f*0.005));
        
        x_swing{s}(f) = ankle_right{s}(1) + res(1)*(x_est{s}(f) - ankle_right{s}(1))*(f*0.005/T)...
                    + res(2)*(x_est{s}(f) - ankle_right{s}(1))*(f*0.005/T)^2 ...
                    + (1-  res(1) - res(2))*(x_est{s}(f) - ankle_right{s}(1))*(f*0.005/T)^3;
    
    end
    plot(x_swing{s})
    hold on
end

% Actual Foot Placement
step_length = zeros(length(pelvis_left), 1);

filted_right_joint = filtfilt(b, a, right_joint(:, 2:3))*pi/180; % filter and transfer to rad


for s = 1:min_steps -1
    
    step_length(s) = -ls*sin(filted_left_joint(left_to_right_index(s, 3), 1) ...
                        - filted_left_joint(left_to_right_index(s, 3), 2)) ...
                        - lt*sin(filted_left_joint(left_to_right_index(s, 3), 1))...
                        + lt*sin(filted_right_joint(left_to_right_index(s, 3), 1))...
                        + ls*sin(filted_right_joint(left_to_right_index(s, 3), 1)...
                            - filted_right_joint(left_to_right_index(s, 3), 2));
                          
    plot(length(pelvis_left{s}), step_length(s), 'k*')
    hold on
              
end
axis([0, 150, -0.6, 0.8])
legend('Estimated swing motion', 'Actual landing location')

ylabel('Swing Motion (m)')
xlabel('Nodes (200Hz)')
hold off

%% Calcualte swing ankle motion relative to pelvis postion
figure()

swing_location = {};

for s = 1:min_steps -1
    
    swing_location{end+1} =  x_swing{s} - pelvis_left{s};
                   
    plot(swing_location{s})
    hold on
              
end


%%
function y = nploynimal(x)

    global min_steps ankle_right

    rms = 0;

    for s = 1 : min_steps - 1

        traj = ankle_right{s};

        T = length(traj)*0.005 - 0.005;

        t = (0:0.005:T)';

        y_npolynimal = traj(1) + x(1)*(traj(end) - traj(1)).*(t./T)...
                        + x(2)*(traj(end) - traj(1)).*(t./T).^2 ...
                        + (1-  x(1) - x(2))*(traj(end) - traj(1)).*(t./T).^3;

        rms = rms + sum((traj - y_npolynimal).^2);

    end

    y = rms/min_steps;
end

function y = nploynimal_y(x)

    global min_steps ankle_right_y

    rms = 0;

    for s = 1 : min_steps - 1

        traj = ankle_right_y{s};

        T = length(traj)*0.005 - 0.005;

        t = (0:0.005:T)';

        y_npolynimal = traj(1) + x(1)*(traj(end) - traj(1)).*(t./T)...
                        + x(2)*(traj(end) - traj(1)).*(t./T).^2 ...
                        + x(3)*(traj(end) - traj(1)).*(t./T).^3 ...
                        + x(4)*(traj(end) - traj(1)).*(t./T).^4 ...
                        + (1-  x(1) - x(2) - x(3) - x(4))*(traj(end) - traj(1)).*(t./T).^5;

        rms = rms + sum((traj - y_npolynimal).^2);

    end

    y = rms/min_steps;
end
                    
