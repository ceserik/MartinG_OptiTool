function [points] = calcAutocrossScore(T_team,T_max)
% According to 2023 fsg rules v1.1

T_max = T_max*1.25;
points = 95.5*(T_max/T_team-1)/0.25;
points = max(points,0) + 4.5;
end

