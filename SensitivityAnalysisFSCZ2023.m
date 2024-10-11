


addpath('C:\Program Files\casadi-3.6.5-windows64-matlab2018b')
cd ..
addpath(genpath('.'));
import casadi.*
cd MartinG_OptiTool


car = carCreate("HAFO24",0,"4WDTV");
% 
% testCar = car;
% [timeBaseline,pointsBaseline, dataBaseline] = simFSCZ2023(testCar);
% results.times.timesBaseline  = timeBaseline;
% results.points.pointsBaseline = pointsBaseline;
% results.data.data.dataBaseline   = dataBaseline;
% save("dataSensitiv_AutoX_Fix","results");
% 
% testCar = car;
% testCar.m = car.m*0.95;
% [timeMass05,pointsMass05, dataMass05] = simFSCZ2023(testCar);
% results.times.timesMass5  = timeMass05;
% results.points.pointsMass5 = pointsMass05;
% results.data.data.dataMass5   = dataMass05;
% save("dataSensitiv_AutoX_Fix2","results");
% 
% testCar = car;
% testCar.Izz = car.Izz*0.95;
% [timeIzz05,pointsIzz05, dataIzz05] = simFSCZ2023(testCar);
% results.times.timesIzz05  = timeIzz05;
% results.points.pointsIzz05 = pointsIzz05;
% results.data.data.dataIzz05   = dataIzz05;
% save("dataSensitiv_AutoX_Fix3","results");
% 
% testCar = car;
% testCar.COGz = car.COGz*0.95;
% [timeCOGz05,pointsCOGz05, dataCOGz05] = simFSCZ2023(testCar);
% results.times.timesCOGz05  = timeCOGz05;
% results.points.pointsCOGz05 = pointsCOGz05;
% results.data.data.dataCOGz05   = dataCOGz05;
% save("dataSensitiv_AutoX_Fix6","results");
% 

testCar = car;
testCar.COGr = car.COGr*0.95;
testCar.COG  = car.wheelbase*(car.COGr); % distance of Center Of Gravity (COG) from the front axle [m];
car.l_f      = testCar.COGr;                  % distance of the front axle form the COG [m];
car.l_r       = testCar.wheelbase - testCar.COGr;  % distance of the rear axle form the COG [m];Mf_maxgear
[timeCOGr05,pointsCOGr05, dataCOGr05] = simFSCZ2023(testCar);
results.times.timesCOGr05  = timeCOGr05;
results.points.pointsCOGr05 = pointsCOGr05;
results.data.data.dataCOGr05   = dataCOGr05;
save("dataSensitiv_AutoX_Fix13_fixCOGr","results");
% 
% %% dalsie urobit 2WD a ubrat zopar kil
% testCar = car;
% testCar.PowertrainType = "2WD";
% testCar.m = car.m - 11;
% [time2WD05,points2WD05, data2WD05] = simFSCZ2023(testCar);
% results.times.times2WD05  = time2WD05;
% results.points.points2WD05 = points2WD05;
% results.data.data.data2WD05   = data2WD05;
% save("dataSensitiv_AutoX_Fix8","results");
% 
% 
% 
% %% gdpr
% testCar = carCreate("HAFO24",1,"4WDTV");
% [timegepd05,pointsgepd05, datagepd05] = simFSCZ2023(testCar);
% results.times.timesgepd05  = timegepd05;
% results.points.pointsgepd05 = pointsgepd05;
% results.data.data.datagepd05   = datagepd05;
% %save("dataSensitiv_AutoX_Fix10","results");
% 
% %% CL
% car = carCreate("HAFO24",0,"4WDTV");
% testCar = car;
% testCar.CL = car.CL * 0.95;
% [timeCL05,pointsCL05, dataCL05] = simFSCZ2023(testCar);
% results.times.timesCL05  = timeCL05;
% results.points.pointsCL05 = pointsCL05;
% results.data.data.dataCL05   = dataCL05;
% save("dataSensitiv_AutoX_Fix11","results");
% 
% %% CD
% car = carCreate("HAFO24",0,"4WDTV");
% testCar = car;
% testCar.CD = car.CD * 0.95;
% [timeCD05,pointsCD05, dataCD05] = simFSCZ2023(testCar);
% results.times.timesCD05  = timeCD05;
% results.points.pointsCD05 = pointsCD05;
% results.data.data.dataCD05   = dataCD05;
% save("dataSensitiv_AutoX_Fix11","results");
% 
% 
% %% GEPD strategy??
% %% battery capacity
% % % track wheelbase gear ratio

