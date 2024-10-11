car = carCreate("HAFO24");

%% sim 2WD car with only diff
car.PowertrainType     = "2WD";
[times2WD,points2WD,data2WD] = simFSCZ2023(car);
results.times.times2WD  = times2WD;
results.points.points2WD = points2WD;
results.data.data.data2WD   = data2WD;
%% sim 4WD car with only diff
car.PowertrainType     = "4WD";
[times4WD,points4WD,data4WD] = simFSCZ2023(car);
results.times.times4WD  = times4WD;
results.points.points4WD = points4WD;
results.data.data.data4WD   = data4WD;
save("4WD","results");
 %% sim 4WDTV car with TV
 car = carCreate("HAFO24");
 car.PowertrainType                 = "4WDTV";
 [times4WDTV,points4WDTV,data4WDTV] = simFSCZ2023(car);
 results.times.times4WDTV           = times4WDTV;
 results.points.points4WDTV         = points4WDTV;
 results.data.data.data4WDTV        = data4WDTV;
 save("4WDTV","results");

 %% sim 4WD car with Rear wheel steering
 car = carCreate("HAFO24");
 car.PowertrainType                 = "4WD";
 car.steeredAxle                    = 'both';
 [times4WDRS,points4WDRS,data4WDRS] = simFSCZ2023(car);
 results.times.times4WDRS           = times4WDRS;
 results.points.points4WDRS         = points4WDRS;
 results.data.data.data4WDRS        = data4WDRS;
  save("4WDRS","results");
%  % 
%  %% sim 4WD car with rear wheel steering
%  car = carCreate("HAFO24");
% car.PowertrainType     = "4WDTV";
% car.steeredAxle = 'both';
