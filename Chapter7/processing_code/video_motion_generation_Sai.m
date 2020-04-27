clc
clear
close all

indego_path = '../Sai/Indego/swing_control/';
cortex_path = '../Sai/Cortex/swing_control/';


% load walking data
trial = 2;
if trial == 2
    t_st = 635*2 + 1;
    t_ed = t_st + 15000*2;
elseif trial == 3
    t_st = 0*2 + 1;
    t_ed = t_st + 15000*2;
end

% Process Simulink Recorded data
motion_indego = load(strcat(indego_path, 'indego_trial_0', num2str(trial), '.mat'));

indego_time = motion_indego.leftFusedHipKnee.time(t_st:t_ed) ...
              - motion_indego.leftFusedHipKnee.time(t_st);
          
left_joint_absolute = [motion_indego.leftThighAngleVelocity.signals(1).values(t_st:t_ed),...
              motion_indego.leftFusedHipKnee.signals(2).values(t_st:t_ed)];
          
right_joint_absolute = [motion_indego.rightThighAngleVelocity.signals(1).values(t_st:t_ed),...
              motion_indego.rightFusedHipKnee.signals(2).values(t_st:t_ed)];
          
grf_cortex = importdata(strcat(cortex_path, 'Mocap000', num2str(trial), '.txt'));

cortex_time = grf_cortex.data(:, 1) - grf_cortex.data(1, 1);

Fy1 = grf_cortex.data(:, 7);
Fy2 = grf_cortex.data(:, 16);

% smooth motions
[b1, a1] = butter(2, 10/(200/2));
left_joint_absolute = filtfilt(b1, a1, left_joint_absolute)*pi/180;
right_joint_absolute = filtfilt(b1, a1, right_joint_absolute)*pi/180;

absolute_joint = [indego_time, left_joint_absolute, right_joint_absolute];

% smooth grf 
[b2, a2] = butter(2, 10/(100/2));
Fy1 = filtfilt(b2, a2, Fy1);
Fy2 = filtfilt(b2, a2, Fy2);

% sychronize ground reaction force data to the time frame of Indego
Fy1_indego = interp1(cortex_time, Fy1, indego_time, 'linear', 'extrap');
Fy2_indego = interp1(cortex_time, Fy2, indego_time, 'linear', 'extrap');

Fy = [indego_time, Fy1_indego, Fy2_indego];

% figure()
% subplot(2,1,1)
% title('Left Leg')
% plot(indego_time, left_joint_absolute(:, 1), '-')
% hold on
% plot(indego_time, left_joint_absolute(:, 2), '--')
% hold on
% plot(indego_time, Fy1_indego*0.001);
% ylabel('Joint Angles (deg)')
% legend('Hip Joint', 'Knee Joint', 'Fy1')
% hold off
% 
% subplot(2,1,2)
% title('Right Leg')
% plot(indego_time, right_joint_absolute(:, 1), '-')
% hold on
% plot(indego_time, right_joint_absolute(:, 2), '--')
% hold on
% plot(indego_time, Fy2_indego*0.001);
% ylabel('Joint Angles (deg)')
% legend('Hip Joint', 'Knee Joint', 'Fy2')
% hold off

%% get estimated foot placement

sim('../Sai/simulink_control_models/swing_control/Gen2MatlabController_200_PCI6014_CT_PO2.slx')

estimated_fp = estimated_foot_placement.data;

swing_path_x = reshape(swing_point.Data(1, 1, :), 20001, 1);
swing_path_y = reshape(swing_point.Data(1, 2, :), 20001, 1);

swing_path_x0 = reshape(swing_point0.Data(1, 1, :), 20001, 1);
swing_path_y0 = reshape(swing_point0.Data(1, 2, :), 20001, 1);

swing0 = [swing_path_x0, swing_path_y0];

pelvis_x = reshape(pelvis_motion.Data(1, 1, :), 20001, 1);

stance_x0 = stance_x.data;
stance_y0 = stance_y.data;
swing_t = swing_t.data;

path_t = [linspace(0.6, 0.6, 121)', (0.6:-0.005:0.0)'];

coe_swingx = [0.1795, 2.4645];
coe_swingy = [6.0075, 5.4484, -64.0954, 95.0540];
%% Plot annimation

ls = 0.35;
lt = 0.41;

shoulder_point = [0, 1.2];

hip_point = [0, 0.73];

video_st = 3065*2;
video_ed = video_st + 1200*2;

knee_left = hip_point + [lt*sin(left_joint_absolute(video_st:video_ed, 1)), -lt*cos(left_joint_absolute(video_st:video_ed, 1))];
ankle_left = knee_left + [ls*sin(left_joint_absolute(video_st:video_ed, 1)...
    - left_joint_absolute(video_st:video_ed, 2)),...
    -ls*cos(left_joint_absolute(video_st:video_ed, 1) - left_joint_absolute(video_st:video_ed, 2))];

knee_right = hip_point + [lt*sin(right_joint_absolute(video_st:video_ed, 1)), -lt*cos(right_joint_absolute(video_st:video_ed, 1))];
ankle_right = knee_right + [ls*sin(right_joint_absolute(video_st:video_ed, 1)...
    - right_joint_absolute(video_st:video_ed, 2)),...
    -ls*cos(right_joint_absolute(video_st:video_ed, 1)...
    - right_joint_absolute(video_st:video_ed, 2))];

% Initialize video with name
walking_video = VideoWriter(strcat(indego_path, 'walking_annimation_', num2str(trial))); %open video file
walking_video.FrameRate = 100;  %can adjust this, 5 - 10 works well for me
open(walking_video)


sign = 1;

% plot frames in the video
for i = 1:1:video_ed - video_st
   
   plot([-1.0, 1.5], [-0.02, -0.02], 'Color', [170 170 170]/255, 'linewidth', 7)
   hold on
   plot([shoulder_point(1), hip_point(1)], [shoulder_point(2), hip_point(2)], 'k-', 'linewidth', 2.5)
   hold on
   plot([hip_point(1), knee_left(i, 1)], [hip_point(2), knee_left(i, 2)], 'r-', 'linewidth', 2.5) 
   hold on
   plot([knee_left(i, 1), ankle_left(i, 1)], [knee_left(i, 2), ankle_left(i, 2)], 'r-', 'linewidth', 2.5)
   hold on
   plot([hip_point(1), knee_right(i, 1)], [hip_point(2), knee_right(i, 2)], 'b-', 'linewidth', 2.5) 
   hold on
   plot([knee_right(i, 1), ankle_right(i, 1)], [knee_right(i, 2), ankle_right(i, 2)], 'b-', 'linewidth', 2.5)
   axis([-1.0, 1.5, -0.1, 1.4])
   hold on
   
   if (swing_t(video_st + i, 2) - swing_t(video_st + i - 1, 2)) >= 0.5
       sign = sign + 1;
   elseif ((swing_t(video_st + i - 1, 2) - swing_t(video_st + i - 2, 2)) <= -0.004 && ...
           swing_t(video_st + i - 1, 2) <= 1e-6)
       sign = sign + 1;
   end
   
   if rem(round(sign/2)-1, 2) == 0 && (round(sign/2) ~= sign/2)
%    if (swing_t(video_st + i, 2) - swing_t(video_st + i - 1, 2) == 0 ...
%            || swing_t(video_st + i + 1, 2) - swing_t(video_st + i, 2) == 0)
   
   path = swing_path(coe_swingx, coe_swingy, stance_x0(video_st + i),...
                     stance_y0(video_st + i), swing0(video_st+i, :), path_t,...
                     estimated_fp(video_st + i));
   
   plot(path(:, 1), path(:, 2) + 0.73, 'b--', 'linewidth', 1.5)
   hold on
   plot(path(1, 1), path(1, 2) + 0.73, 'bo')
   hold on
   plot(path(end, 1), path(end, 2) + 0.73, 'bo')
   
   elseif rem(round(sign/2)-1, 2) == 1 && (round(sign/2) ~= sign/2)
       
   path = swing_path(coe_swingx, coe_swingy, stance_x0(video_st + i),...
                     stance_y0(video_st + i), swing0(video_st+i, :), path_t,...
                     estimated_fp(video_st + i));
   
   plot(path(:, 1), path(:, 2) + 0.73, 'r--', 'linewidth', 1.5)
   hold on
   plot(path(1, 1), path(1, 2) + 0.73, 'ro')
   hold on
   plot(path(end, 1), path(end, 2) + 0.73, 'ro')
   end
   hold off
   pause(0.005)
   
   frame = getframe(gcf); %get frame
   writeVideo(walking_video, frame);
end

close(walking_video)

%% generate swing path

function path = swing_path(coe_swingx, coe_swingy, stance_x,...
                    stance_y, swing0, time, predict)
         
    swing_tp = time(:, 1);
    swing_t = time(:, 1) - time(:, 2);
     
    B = coe_swingx(1);
    C = coe_swingx(2);
    
    By = coe_swingy(1);
    Cy = coe_swingy(2);
    Dy = coe_swingy(3);
    Ey = coe_swingy(4);
    
    
    Scale = predict - swing0(1);
    
    Scale_y = 0.05 - swing0(2);
    
    ScaleT = (swing_t)./(swing_tp);

    swing_x = (swing0(1) + B*Scale*ScaleT + C*Scale*ScaleT.^2 ... 
                 + (1 - B - C)*Scale*ScaleT.^3) - stance_x; 
             
    swing_y = (swing0(2) + By*Scale_y*ScaleT + Cy*Scale_y*ScaleT.^2 ... 
                 + Dy*Scale_y*ScaleT.^3 + Ey*Scale_y*ScaleT.^4 ...
                 + (1 - By - Cy - Dy - Ey)*Scale_y*ScaleT.^5) - stance_y; 
             
    path = [swing_x, swing_y];
end





















