%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to calculate parameters of each subjects in the experiments.
% The parameters including segement mass, inertial, length. These
% parameters were calculated using quiet standing recorded data, and some 
% measured parameters.

% Input:
%       QS_M: marker data of quiet standing trial
%       Mass: mass of participant (kg)
%       Height: height of participant (cm)

%       Parameters: segement length, mass, inertia, and center of mass
%       Order of the parameters are: 
%       1. length of lower leg
%       2. length of upper leg
%       3. center of mass of lower leg
%       4. center of mass of upper leg
%       5. center of mass of trunk
%       6. mass of lower leg
%       7. mass of upper leg
%       8. mass of trunk
%       9. inertia of lower leg
%       10. inertia of upper leg
%       11. inertia of trunk

%       12/26/2017
%       Huawei Wang
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function Parameters = Model_Parameter_Calculate(QS_M, Mass, Height,...
                                                knee_width, ankle_width)

    l_F = (mean(QS_M(:, 43)-QS_M(:, 49)) + mean(QS_M(:, 67)-QS_M(:, 73)))/2 + 0.015;
    l_L = (mean(QS_M(:, 37)-QS_M(:, 43)) + mean(QS_M(:, 61)-QS_M(:, 67)))/2;
    l_U = (mean(QS_M(:, 31)-QS_M(:, 37)) + mean(QS_M(:, 55)-QS_M(:, 61)))/2;
    l_T = (mean(QS_M(:, 82)-QS_M(:, 31)) + mean(QS_M(:, 79)-QS_M(:, 55)))/2;
    l_H = Height*0.01 - l_L - l_U - l_T - l_F - 0.02;

    w_P = (mean(QS_M(:, 32)-QS_M(:, 56)));
    w_S = (mean(QS_M(:, 83)-QS_M(:, 80)));
    w_T = (w_P + w_S)/2;
    w_H = w_P/2;
    w_K = knee_width*0.01;
    w_A = ankle_width*0.01;

    d_T = (mean(QS_M(:, 3)+QS_M(:, 6))/2 - mean(QS_M(:, 9)+QS_M(:, 12)+QS_M(:, 15))/3);

    v_L = 2/3*pi*l_L*((w_A/2)^2 + (w_A/2)*(w_K/2) + (w_K/2)^2);
    v_U = 2/3*pi*l_L*((w_K/2)^2 + (w_K/2)*(w_H/2) + (w_H/2)^2);
    v_T = (d_T*(w_T-d_T) + 1/4*pi*d_T^2)*l_T;
    v_H = 4/3*pi*((l_H)/2)^3;

    total_v = v_L + v_U + v_T + v_H;

    m_L = v_L/total_v*Mass*0.97;
    m_U = v_U/total_v*Mass*0.97;
    m_T = v_T/total_v*Mass*0.97;
    m_H = v_H/total_v*Mass*0.97;

    d_L = w_K/(w_A+w_K)*l_L;
    d_U = w_H/(w_K+w_H)*l_U;
    d_T = m_H/(m_T+m_H)*(l_T+l_H)/2 + l_T/2;

    % Calculate the inertia of the lower leg

    rm_L = w_A/2 + d_L/l_L*(w_K/2-w_A/2);

    v_LU = 1/3*pi*(l_L-d_L)*(rm_L^2 + rm_L*w_K/2 + (w_K/2)^2);
    v_LD = 1/3*pi*(d_L)*((w_A/2)^2 + w_A/2*rm_L + rm_L^2);

    m_LU = v_LU/(v_LU + v_LD)*m_L;
    m_LD = v_LD/(v_LU + v_LD)*m_L;

    d_LU = (w_K/2)/(rm_L + w_K/2)*(l_L-d_L);
    d_LD = rm_L/(rm_L + w_A/2)*d_L;

    i_L = m_LU*d_LU^2 + m_LD*(d_L - d_LD)^2;


    % Calculate Upper leg inertia

    rm_U = w_K/2 + d_U/l_U*(w_H/2-w_K/2);

    v_UU = 1/3*pi*(l_U-d_U)*(rm_U^2 + rm_U*w_H/2 + (w_H/2)^2);
    v_UD = 1/3*pi*(d_U)*((w_K/2)^2 + w_K/2*rm_U + rm_U^2);

    m_UU = v_UU/(v_UU + v_UD)*m_U;
    m_UD = v_UD/(v_UU + v_UD)*m_U;

    d_UU = (w_H/2)/(rm_U + w_H/2)*(l_U-d_U);
    d_UD = rm_U/(rm_U + w_K/2)*d_U;

    i_U = m_UU*d_UU^2 + m_UD*(d_U - d_UD)^2;


    % Calculate inertia of torso and head

    d_TU = l_H/2 + l_T - d_T;
    d_TD = l_T/2;

    i_T = m_H*d_TU^2 + m_T*(d_T - d_TD)^2;

    Parameters = [l_L, l_U, d_L, d_U, d_T, m_L, m_U, m_T, i_L, i_U, i_T];

end




