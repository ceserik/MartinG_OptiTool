import casadi.*
%profile clear;
%profile on;
addpath('/home/Riso/Downloads/casadimojemoje/')
car = carCreate("HAFO24",0,"4WD");

[time,expdata,opti,sol] = runLaptime(car,"doubleTurn");
%#profile off;
%profile viewer;