

bestTimes = [ 4.187 2.9830 53.9059 55.764 ];

results.points.pointsBaseline = recalc_scores(results.data.data.dataBaseline,bestTimes);
results.points.pointsgepd05   = recalc_scores(results.data.data.dataGEPD,bestTimes);



%% calc deltas

deltaGEPD = abs(results.points.pointsBaseline.Total   - results.points.pointsgepd05.Total)/438.5*100;





deltaGEPDAccel = abs(results.points.pointsBaseline.Acceleration   - results.points.pointsgepd05.Acceleration)/46.5*100;



deltaGEPDSkidpad = abs(results.points.pointsBaseline.Skidpad   - results.points.pointsgepd05.Skidpad)/46.5*100;



deltaGEPDAutocross = abs(results.points.pointsBaseline.Autocross   - results.points.pointsgepd05.Autocross)/95.5*100;



deltaGEPDEndurance = abs(results.points.pointsBaseline.Endurance   - results.points.pointsgepd05.Endurance)/250*100;





% Initialize data points
D1 = [deltaGEPDEndurance];
D2 = [deltaGEPDEndurance];
D3 = [deltaGEPDEndurance ];
D4 = [deltaGEPDEndurance];
D5 = [deltaGEPDEndurance  ];
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

