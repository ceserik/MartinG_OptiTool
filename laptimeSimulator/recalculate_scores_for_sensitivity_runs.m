

bestTimes = [ 4.187 2.9830 53.9059 55.764 ];

results.points.points2WD05    = recalc_scores(results.data.data.data2WD05,bestTimes);
results.points.pointsBaseline = recalc_scores(results.data.data.dataBaseline,bestTimes);
results.points.pointsCOGr05   = recalc_scores(results.data.data.dataCOGr05,bestTimes);
results.points.pointsCOGz05   = recalc_scores(results.data.data.dataCOGz05,bestTimes);
results.points.pointsgepd05   = recalc_scores(results.data.data.datagepd05,bestTimes);
results.points.pointsIzz05    = recalc_scores(results.data.data.dataIzz05,bestTimes);
results.points.pointsMass5    = recalc_scores(results.data.data.dataMass5,bestTimes);
results.points.pointsCL05     = recalc_scores(results.data.data.dataCL05,bestTimes);
results.points.pointsCD05     = recalc_scores(results.data.data.dataCD05,bestTimes);


%% calc deltas
delta2WD = abs(results.points.pointsBaseline.Total   - results.points.points2WD05.Total)/438.5*100;
deltaCOGr = abs(results.points.pointsBaseline.Total   - results.points.pointsCOGr05.Total)/438.5*100;
deltaCOGz = abs(results.points.pointsBaseline.Total   - results.points.pointsCOGz05.Total)/438.5*100;
deltaGEPD = abs(results.points.pointsBaseline.Total   - results.points.pointsgepd05.Total)/438.5*100;
deltaIzz = abs(results.points.pointsBaseline.Total   - results.points.pointsIzz05.Total)/438.5*100;
deltaMass = abs(results.points.pointsBaseline.Total   - results.points.pointsMass5.Total)/438.5*100;
deltaCL = abs(results.points.pointsBaseline.Total   - results.points.pointsCL05.Total)/438.5*100;
deltaCD = abs(results.points.pointsBaseline.Total   - results.points.pointsCD05.Total)/438.5*100;


delta2WDAccel  = abs(results.points.pointsBaseline.Acceleration   - results.points.points2WD05.Acceleration)/46.5*100;
deltaCOGrAccel = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsCOGr05.Acceleration)/46.5*100;
deltaCOGzAccel = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsCOGz05.Acceleration)/46.5*100;
deltaGEPDAccel = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsgepd05.Acceleration)/46.5*100;
deltaIzzAccel  = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsIzz05.Acceleration)/46.5*100;
deltaMassAccel = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsMass5.Acceleration)/46.5*100;
deltaCLAccel   = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsCL05.Acceleration)/46.5*100;
deltaCDAccel   = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsCD05.Acceleration)/46.5*100;


delta2WDSkidpad  = abs(results.points.pointsBaseline.Skidpad   - results.points.points2WD05.Skidpad)/46.5*100;
deltaCOGrSkidpad = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsCOGr05.Skidpad)/46.5*100;
deltaCOGzSkidpad = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsCOGz05.Skidpad)/46.5*100;
deltaGEPDSkidpad = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsgepd05.Skidpad)/46.5*100;
deltaIzzSkidpad  = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsIzz05.Skidpad)/46.5*100;
deltaMassSkidpad = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsMass5.Skidpad)/46.5*100;
deltaCLSkidpad   = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsCL05.Skidpad)/46.5*100;
deltaCDSkidpad   = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsCD05.Skidpad)/46.5*100;

delta2WDAutocross  = abs(results.points.pointsBaseline.Autocross   - results.points.points2WD05.Autocross)/95.5*100;
deltaCOGrAutocross = abs(results.points.pointsBaseline.Autocross   - results.points.pointsCOGr05.Autocross)/95.5*100;
deltaCOGzAutocross = abs(results.points.pointsBaseline.Autocross   - results.points.pointsCOGz05.Autocross)/95.5*100;
deltaGEPDAutocross = abs(results.points.pointsBaseline.Autocross   - results.points.pointsgepd05.Autocross)/95.5*100;
deltaIzzAutocross  = abs(results.points.pointsBaseline.Autocross   - results.points.pointsIzz05.Autocross)/95.5*100;
deltaMassAutocross = abs(results.points.pointsBaseline.Autocross   - results.points.pointsMass5.Autocross)/95.5*100;
deltaCLAutocross   = abs(results.points.pointsBaseline.Autocross   - results.points.pointsCL05.Autocross)/95.5*100;
deltaCDAutocross   = abs(results.points.pointsBaseline.Autocross   - results.points.pointsCD05.Autocross)/95.5*100;

delta2WDEndurance  = abs(results.points.pointsBaseline.Endurance   - results.points.points2WD05.Endurance)/250*100;
deltaCOGrEndurance = abs(results.points.pointsBaseline.Endurance   - results.points.pointsCOGr05.Endurance)/250*100;
deltaCOGzEndurance = abs(results.points.pointsBaseline.Endurance   - results.points.pointsCOGz05.Endurance)/250*100;
deltaGEPDEndurance = abs(results.points.pointsBaseline.Endurance   - results.points.pointsgepd05.Endurance)/250*100;
deltaIzzEndurance  = abs(results.points.pointsBaseline.Endurance   - results.points.pointsIzz05.Endurance)/250*100;
deltaMassEndurance = abs(results.points.pointsBaseline.Endurance   - results.points.pointsMass5.Endurance)/250*100;
deltaCLEndurance   = abs(results.points.pointsBaseline.Endurance   - results.points.pointsCL05.Endurance)/250*100;
deltaCDEndurance   = abs(results.points.pointsBaseline.Endurance   - results.points.pointsCD05.Endurance)/250*100;




% Initialize data points
D1 = [deltaMass deltaCOGr deltaCOGz deltaIzz deltaCL deltaCD];
D2 = [deltaMassAccel deltaCOGrAccel deltaCOGzAccel deltaIzzAccel deltaCLAccel deltaCDAccel];
D3 = [deltaMassSkidpad deltaCOGrSkidpad deltaCOGzSkidpad deltaIzzSkidpad deltaCLSkidpad deltaCDSkidpad ];
D4 = [deltaMassAutocross deltaCOGrAutocross deltaCOGzAutocross deltaIzzAutocross deltaCLAutocross deltaCDAutocross ];
D5 = [deltaMassEndurance deltaCOGrEndurance deltaCOGzEndurance deltaIzzEndurance deltaCLEndurance deltaCDEndurance ];
P = [D5;D2;D3;D4;D1];
% Delete variable in workspace if exists
if exist('s', 'var')
        % delete(s);
end
%% Spider plot
figure(1)
s = spider_plot_class(P);
s.AxesLimits = [zeros(1,length(D1)); ones(1,length(D1))*max([D1,D2,D3,D4])]; % [min axes limits; max axes limits]
% Legend properties
s.AxesLabels = {"Mass -", " COG X -", "COG Z -", "Inertia +", "Lift coeffcient +","Drag coefficient -"};
s.LegendLabels = {'Endurance+Efficiency score impact','Acceleration score impact','Skidpad score impact','Autocross score impact','Total score impact'};
s.LegendHandle.Location = 'northeastoutside';
s.FillOption = 'on';
s.AxesInterpreter ='latex';
s.LabelFontSize=11;
s.AxesFontSize=11;
s.LegendHandle.FontSize =11;
s.LegendHandle.Interpreter = 'latex';
s.LegendHandle.Location="best";
%set(gcf, 'Position',  [0, 0, 650, 350])
title("Impact of 5% parameter change on points gained");

%% bar plot
figure(2)

 
 bar(["Mass -", " COG X -", "COG Z -", "Inertia +", "Lift coeffcient +","Drag coefficient -"],D1)
 grid on
 ylabel("Points difference")
 title("Sensitivity analysis of 5% parameter change to gain point")


 %% bar plot for gepd and 2WD
 figure(3)
 bar(["GEPD" "2WDvs4WD"],[deltaGEPD delta2WD].*438.5/100)
 grid on
 ylabel("Points difference")
 title("GEPD and 4WD influence on laptime")

