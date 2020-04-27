clc
clear
close all

data_path_indego = '..\jinfeng\Indego\swing_control\';

trials = [2, 3, 4, 5, 6, 7];

for trial = trials
    
    indego = load(strcat(data_path_indego, 'indego_trial_0', ...
                            num2str(trial) ,'.mat'));

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

    right_step_index = importdata(strcat(data_path_indego, 'motion_hs_to_index_right_trial0', num2str(trial),'.txt'));

    step_normal = zeros(200, 2, length(right_step_index(:, 1)));

    for s = 1:length(right_step_index(:, 1))

        step = right_joint_absolute(right_step_index(s, 1):right_step_index(s, 3) ,:);
        time_indego = (0:0.005:(right_step_index(s, 3) - right_step_index(s, 1))*0.005)';
        time_normal = linspace(0, (right_step_index(s, 3) - right_step_index(s, 1))*0.005, 200)';
        step_normal(:, :, s) = interp1(time_indego, step, time_normal, 'linear', 'extrap');  

    end

    mean_step = mean(step_normal, 3);
    std_step = std(step_normal, 0, 3);

    fid = fopen(strcat(data_path_indego, 'Mean_step', num2str(trial), '.txt'), 'w');

    for iy = 1:length(mean_step(:, 1))
        fprintf(fid,'%d ', mean_step(iy, :));  % then the data
        fprintf(fid, '\n');
    end
    fclose(fid);

    fid = fopen(strcat(data_path_indego, 'Std_step', num2str(trial), '.txt'), 'w');

    for iy = 1:length(std_step(:, 1))
        fprintf(fid,'%d ', std_step(iy, :));  % then the data
        fprintf(fid, '\n');
    end
    fclose(fid);

end
