%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code will fill the gap of the marker data using the spline
% interplate function and gives out the percentage of gaps in the recorded
% data.
% 

% Input: 
%          raw marker data

% Output: 
%          filled marker data
%          percentage of gaps
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [filled_data, IZero] = FillMarkerGap(raw_data)
    
    RL = length(raw_data(1,:));
    CL = length(raw_data(:,1));
    IZero = zeros(2, RL - 2);

    Time = raw_data(:, 1);

    for k = 3 : RL
        time_RL = Time;
        marker_RL = raw_data(:, k);
        Ind = find(marker_RL==0);

        if ~isempty(Ind)

            IZero(1, k-2) = length(Ind)/CL;

            Gap_max = 1;
            Gap = 1;
            for q = 1:length(Ind)-1
                if Ind(q+1) == Ind(q) + 1
                    Gap = Gap + 1;
                else
                    Gap = 1;
                end
                if Gap > Gap_max
                    Gap_max = Gap;
                end
            end

            IZero(2, k-2) = Gap_max/100.0;

            time_RL(Ind) = [];
            marker_RL(Ind) = [];

            raw_data(Ind, k) = interp1(time_RL, marker_RL, Time(Ind), 'pchip', 'extrap');
        end
    end
    
    filled_data = raw_data;

end

