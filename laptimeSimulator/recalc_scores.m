function points = recalc_scores(data,minTimes)
%RECALC_SCORES realcualtes score of each discipline according to minimum
%times
%% define best times for events
BestAccelTime     = minTimes(2); 
BestSkidpadTime   = minTimes(1);
BestAutocrossTime = minTimes(3); 
BestEnduranceTime = minTimes(4);


%% extract times from data
[skidpadScore] = calcSkidpadTimeScore(BestSkidpadTime,data(:,1));
timeAccel = data(2).z(end,6);
timeEndurance = data(3).z(end,6);
timeAutocross = data(4).z(end,6);

%% calc scores
points.Endurance    = calcEnduranceScore(timeEndurance*22,BestEnduranceTime*22);
points.Skidpad      = skidpadScore;
points.Acceleration = calcAccelerationScore(BestAccelTime,timeAccel);
points.Autocross    = calcAutocrossScore(timeAutocross,BestAutocrossTime);
points.Total        = points.Endurance +points.Skidpad + points.Acceleration + points.Autocross;

end

