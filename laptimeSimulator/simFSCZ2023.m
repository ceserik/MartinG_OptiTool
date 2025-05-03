function [times,points,data] = simFSCZ2023(car)
%Simulates dynamic disciplines from FSCZ2023 and calculates points
cd ..
addpath(genpath('.'));
cd MartinG_OptiTool
addpath('C:\Program Files\casadi-3.6.4-windows64-matlab2018b')


%car.Sus_t = 5;
%% define best times for events
BestAccelTime = 2.9830; % zobrat realnu hodntu lebo pilot nerobi nicbestTimes = [    55.764 ]
BestSkidpadTime = 4.187;
BestAutocrossTime = 53.9059; %TV GEPD rear steering
BestEnduranceTime = 55.764;

%% Skidpad
car.discipline = "skidpad";
[timeSkidpad, dataSkidpad] = runLaptime(car,"skidpad");
[skidpadScore,T_team1,T_team2] = calcSkidpadTimeScore(BestSkidpadTime,dataSkidpad);
optdata(dataSkidpad.z,dataSkidpad.u,car);
%raceCar_visu(dataSkidpad, 'ColoredTrackLine', true, 'ShowTireForce', true);

%% Acceleration
% todo fix low speed simulation :(
car.discipline = "acceleration";
[timeAccel, dataAccel] = runLaptime(car,"accel");

%% Endurance
car.discipline = "endurance";
[timeEndurance,datandu] = runLaptime(car,"FSCZ2023");

%% Autocross
car.discipline = "autoX";
[timeAutocross,dataAutocross] = runLaptime(car,"FSCZ2023");
%% efficiency calc

%% points calc
points.Endurance    = calcEnduranceScore(timeEndurance*22,BestEnduranceTime*22);
points.Skidpad      = skidpadScore;
points.Acceleration = calcAccelerationScore(BestAccelTime,timeAccel);
points.Autocross    = calcAutocrossScore(timeAutocross,BestAutocrossTime);
points.Total        = points.Endurance +points.Skidpad + points.Acceleration + points.Autocross;

times.Endurance     = timeEndurance;
times.Skidpad       = (T_team1 + T_team2)/2;
times.Autocross     = timeAutocross;
times.Acceleration  = timeAccel;

data = [dataSkidpad dataAccel datandu dataAutocross];
end