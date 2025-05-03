function [skidpadScore,T_team1,T_team2] = calcSkidpadTimeScore(T_max,SkidpadData)
%calcualtes skidapd time and score according to fsg2023 v1.1

%detection of crossing of detection point
lastY = SkidpadData.carpos(1,2);
detectedIndexes = 0;
for  i = 2:length(SkidpadData.carpos)
    if lastY <0 && SkidpadData.carpos(i,2) > 0
        detectedIndexes(end+1) = i;
    end
    lastY = SkidpadData.carpos(i,2);

end
% index 2 3 je prve kolecko index 5 6 je druhe kolecko

T_team1 = SkidpadData.z(detectedIndexes(3),6) - SkidpadData.z(detectedIndexes(2),6);
T_team2 = SkidpadData.z(detectedIndexes(6),6) - SkidpadData.z(detectedIndexes(5),6);

T_team = (T_team1+T_team2)/2; 

T_max = T_max * 1.25;
skidpadScore = max(46.5*((T_max/T_team)^2-1)/0.5625,0) + 3.5;
end

