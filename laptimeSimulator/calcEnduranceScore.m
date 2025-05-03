function [points] = calcEnduranceScore(T_team, T_max)
% According to 2023 fsg rules v1.1
P_max = 250;%?
T_max = T_max*1.333;
points = 0.9*P_max*((T_max/T_team)-1)/0.333;
points = max(points,0) + 25;

end

