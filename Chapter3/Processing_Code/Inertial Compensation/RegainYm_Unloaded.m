function F1_regain = RegainYm_Unloaded(Analog)

    CalibrationMatrix = [3.211, -4.296, 9.517, -506.156, -503.077, -12.798;...
                         -488.7, -489.177, -490.137, 12.72, -12.405, 6.997;...
                         -2.439, -22.137, -12.713, -5.808, 0.306, -529.752;...
                         51.205, -296.238, 301.239, 0, 0, 54.628;...
                         0, 0, 0, -252.497, 357.263, -9.06;...
                         249.758, 24.2, 20.303, -52.477, -52.08, 0;];
                     
    F1_regain_all = -Analog*CalibrationMatrix';
    
    time = (0:0.01:(length(Analog(:, 1))-1)*0.01);
    
    F1_regain = interp1(time, F1_regain_all(:, 5), time - 0.01 , 'linear', 'extrap');
   
end

                 
                 
