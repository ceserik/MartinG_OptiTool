function score = calcAccelerationScore(T_max,T_team)
%Calcualte score acceleration according to fsg 2023 v1.1
T_max = T_max*1.5;
score =  max(46.5*(T_max/T_team -1)/0.5,0) + 3.5;

end

