import casadi.*
%profile clear;
%profile on;
addpath('/home/Riso/Downloads/casadimojemoje/')
car = carCreate("HAFO24",0,"2WD");

[time,expdata,opti,sol,mydata] = runLaptime(car,"doubleTurn");
%#profile off;
%profile viewer;