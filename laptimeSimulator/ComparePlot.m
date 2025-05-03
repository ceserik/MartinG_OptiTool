Skidpad = 1;
Accel =2;
Endurance = 3;
AutoX = 4;
discipline = Endurance;


disciplineName = "XDD";
switch discipline
    case 1
        disciplineName = "Skidpad";
    case 2
        disciplineName = "Accel";
    case 3
        disciplineName ="Endurance";
    case 4
        disciplineName = "Autox";
end

full=0;
%urobit web plot na body z disciplin, urobit grafy na koal a skalovat podla
%fZ aby tobola jednicka , uroven trati
%dat tam este rozdiel casu na useku
myblue = [0 0.4470 0.7410];
myred  = [0.8500 0.3250 0.0980];

set(0,'defaulttextinterpreter','latex')
set(groot,'defaultAxesfontsize',11)
track.w_l = 1.0; % Width of the track [m] track width = 3m, car width = 1.2m => singltrack can move +-0.9m zvladlo fscz na 0.3 collocation
track.w_r = 1.0; % Width of the track [m]
track.closed = true;
load tracks/FSCZ2023.mat;
x_smpl  = flip(trackFSCZ.Data.Proc.x);
y_smpl  = flip(trackFSCZ.Data.Proc.y);
smooth_factor = 1e1;
optparams.FSdiscipline = false;
%[x_traj, y_traj, s_traj, th_traj, C_traj] = smoothTrackByOCP(x_smpl, y_smpl, smooth_factor, track.closed, 1);

label1 = "baseline";
label2 = "GEPD";

data1 = results.data.data.data4WD(discipline);
data2 = results.data.data.data4WDgepd(discipline);
mydata1 = optdata(data1.z,data1.u,data1.car);
mydata2 = optdata(data2.z,data2.u,data2.car);


testName = sprintf('Discipline-%s_Vehicle-%s_GEPD%d', disciplineName, data2.car.PowertrainType, data2.car.gepdToggle);
    



if discipline >=3
    start = 1300;
    stop = 1550;%length(data1.u);
else
    start = 1;
    stop = length(data1.u);
end

if discipline == Skidpad
    start =400;
    stop = 546;

end

if full
start = 1
stop =  length(data1.u);
end
f= figure(72);
scale = 3*0.8;
f.Position = [0 0 210*scale 297*scale*1.2];

%% track layout and velocity difference
t = tiledlayout(5,2);

ax1 = nexttile([2 1]);


velocityDifference =  mydata2.vx.data(start:stop)-mydata1.vx.data(start:stop);
axis equal
scatter(data2.carpos(start:stop,1),data2.carpos(start:stop,2),20,velocityDifference,'filled')
a=colorbar;
a.Label.String = 'Velocity difference [m/s]';
a.Label.Interpreter = 'latex';
a.Label.FontSize = 11;
hold on
%plotTrack(results.data.data.data4WDRS(4).track, results.data.data.data4WDRS(4).track.s0:results.data.data.data4WDRS(4).track.sf, 1)
%plot(data1.carpos(start:stop,1),data1.carpos(start:stop,2),LineWidth=2)
%hold on
%plot(data2.carpos(start:stop,1),data2.carpos(start:stop,2),LineWidth=2,LineStyle=":")
scatter(data1.carpos(start,1),data1.carpos(start,2),"red")
scatter(data1.carpos(stop,1),data1.carpos(stop,2),100,"red",Marker="x")
axis equal
grid on
ylabel("Vehicle postion X [m]")
xlabel("Vehicle postion Y [m]")
legend('',"start","stop",'Interpreter','latex')
%% GG diagram

ax2 = nexttile([2 1]);

%[k, ~] = convhull([mydata1.ay.data(start:stop),mydata1.ax.data(start:stop)]);
%plot(mydata1.ay.data(start+k-1), mydata1.ax.data(start+k-1), Color=blue,LineWidth=2)
hold on
scatter(mydata1.ay.data(start:stop),mydata1.ax.data(start:stop),20,myblue,"filled")
hold on


scatter(mydata2.ay.data(start:stop),mydata2.ax.data(start:stop),20,myred,"filled")
%[k2, ~] = convhull([mydata2.ay.data(start:stop),mydata2.ax.data(start:stop)]);
%plot(mydata2.ay.data(start+k2-1), mydata2.ax.data(start+k2-1), Color=red,LineWidth=2)
axis equal
grid on

xdd = legend(label1,label2,'Interpreter','latex');
xdd.Position = ([0.424728690863409,0.594706071038439,0.179226335591617,0.037263159885741])
xlabel("Lateral acceleration $[m/s^2]$")
ylabel("Longintudinal acceleration $[m/s^2]$")
%% steering
ax3 = nexttile;
plot(data1.s(start:stop),mydata1.steering_f.data(start:stop),LineWidth=2)
ylabel("Steering angle front[deg] ",'Interpreter','latex')
xlabel("Position on track [m]")
hold on
plot(data1.s(start:stop),mydata2.steering_f.data(start:stop),':',LineWidth=2)
if data2.car.steeredAxle == "both"
    %plot(data1.s(start:stop),mydata2.steering_r.data(start:stop),':',LineWidth=2)
    %legend('',"front $\delta$","rear $\delta$",'Interpreter','latex','Location','best')
end
grid on
axis tight
ax3.YLimitMethod = 'padded';
%legend(label1,label2)



%% accelerations longitudinal
ax7 = nexttile;
plot(data1.s(start:stop),mydata1.ax.data(start:stop),LineWidth=2)
hold on
plot(data1.s(start:stop),mydata2.ax.data(start:stop),':',LineWidth=2)
grid on
ylabel("Acceleration X $[m/s^2]$")
xlabel("Position on track [m]")
%legend(label1,label2)
axis tight
ax7.YLimitMethod = 'padded';

%% yaw rate
ax4 = nexttile;
plot(data1.s(start:stop),mydata1.dpsi.data(start:stop),LineWidth=2)
hold on
plot(data1.s(start:stop),mydata2.dpsi.data(start:stop),':',LineWidth=2)
grid on
ylabel("Yaw rate [deg/s]")
xlabel("Position on track [m]")
%legend(label1,label2)
axis tight
ax4.YLimitMethod = 'padded';

%% accelerations LATERAL
ax8 = nexttile;
plot(data1.s(start:stop),mydata1.ay.data(start:stop),LineWidth=2)
hold on
plot(data1.s(start:stop),mydata2.ay.data(start:stop),':',LineWidth=2)
grid on
ylabel("Acceleration Y $[m/s^2]$")
xlabel("Position on track [m]")
%legend(label1,label2)
axis tight
ax8.YLimitMethod = 'padded';

%% losses
ax6 = nexttile;
plot(data1.s(start:stop), (mydata1.Fy_fl.data(start:stop) + mydata1.Fy_fr.data(start:stop)) .*sin(mydata1.steering_f.data(start:stop)*pi/180) + (mydata1.Fy_rl.data(start:stop) + mydata1.Fy_rr.data(start:stop)) .*sin(mydata1.steering_r.data(start:stop)*pi/180) - 0* mydata1.Fdrag_aero.data(start:stop),LineWidth=2);
hold on
plot(data1.s(start:stop), (mydata2.Fy_fl.data(start:stop) + mydata2.Fy_fr.data(start:stop)) .*sin(mydata2.steering_f.data(start:stop)*pi/180) +  (mydata2.Fy_rl.data(start:stop) + mydata2.Fy_rr.data(start:stop)) .*sin(mydata2.steering_r.data(start:stop)*pi/180) - 0*mydata2.Fdrag_aero.data(start:stop),LineWidth=2,LineStyle=':');
grid on
ylabel("Induced tire drag $[N]$")
xlabel("Position on track [m]")
%legend(label1,label2)
axis tight
ax6.YLimitMethod = 'padded';
%% velocity
ax5 = nexttile;
plot(data1.s(start:stop),mydata1.vx.data(start:stop),LineWidth=2)
hold on
plot(data1.s(start:stop),mydata2.vx.data(start:stop),':',LineWidth=2)
grid on
ylabel("Speed [$m/s$]")
xlabel("Position on track [m]")
%legend(label1,label2)
axis tight
ax5.YLimitMethod = 'padded';

linkaxes([ ax3 ax4 ax5 ax6 ax7 ax8],'x')
t.Padding = 'compact';
t.TileSpacing = 'compact';
%linkaxes([ax8 ax7],'y')

BigComparison = sprintf('%s%s_%s%s','renders/','BigComparison',testName,'.eps')
saveas(gcf,BigComparison,'epsc')

%% calc tire utilisation
tireUtilisation1_fl = mean(tireUtilisation(mydata1.Fx_fl.data(start:stop),mydata1.Fy_fl.data(start:stop),mydata1.Fz_fl.data(start:stop),mydata1.car));
tireUtilisation1_fr = mean(tireUtilisation(mydata1.Fx_fr.data(start:stop),mydata1.Fy_fr.data(start:stop),mydata1.Fz_fr.data(start:stop),mydata1.car));
tireUtilisation1_rl = mean(tireUtilisation(mydata1.Fx_rl.data(start:stop),mydata1.Fy_rl.data(start:stop),mydata1.Fz_rl.data(start:stop),mydata1.car));
tireUtilisation1_rr = mean(tireUtilisation(mydata1.Fx_rr.data(start:stop),mydata1.Fy_rr.data(start:stop),mydata1.Fz_rr.data(start:stop),mydata1.car));

tireUtilisation1_car = mean ([tireUtilisation1_fl tireUtilisation1_fr tireUtilisation1_rl tireUtilisation1_rr])


tireUtilisation2_fl = mean(tireUtilisation(mydata2.Fx_fl.data(start:stop),mydata2.Fy_fl.data(start:stop),mydata2.Fz_fl.data(start:stop),mydata2.car));
tireUtilisation2_fr = mean(tireUtilisation(mydata2.Fx_fr.data(start:stop),mydata2.Fy_fr.data(start:stop),mydata2.Fz_fr.data(start:stop),mydata2.car));
tireUtilisation2_rl = mean(tireUtilisation(mydata2.Fx_rl.data(start:stop),mydata2.Fy_rl.data(start:stop),mydata2.Fz_rl.data(start:stop),mydata2.car));
tireUtilisation2_rr = mean(tireUtilisation(mydata2.Fx_rr.data(start:stop),mydata2.Fy_rr.data(start:stop),mydata2.Fz_rr.data(start:stop),mydata2.car));

tireUtilisation2_car = mean ([tireUtilisation2_fl tireUtilisation2_fr tireUtilisation2_rl tireUtilisation2_rr])

%% plot tire ellipse
xD = figure(71);

tiledlayout(2,2)
%scatter(mydata2.Fx_fl.data./mydata2.Fz_fl.data, mydata2.Fy_fl.data./mydata2.Fz_fl.data)
nexttile
%maxForceCircle = car.tire_fyFactor*[1 * cos(linspace(0, 2*pi, 200)); car.tire_fxFactor/car.tire_fyFactor*sin(linspace(0, 2*pi, 200))] ;
maxForceCircle = [1 * cos(linspace(0, 2*pi, 200)); sin(linspace(0, 2*pi, 200))] ;


plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Black', 'LineWidth', 2);
hold on
plotTire(mydata1.Fy_fl.data(start:stop),mydata1.Fx_fl.data(start:stop),mydata1.Fz_fl.data(start:stop),myblue)
plotTire(mydata2.Fy_fl.data(start:stop),mydata2.Fx_fl.data(start:stop),mydata2.Fz_fl.data(start:stop),myred)
hold off

nexttile
plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Black', 'LineWidth', 2);
hold on
plotTire(mydata1.Fy_fr.data(start:stop),mydata1.Fx_fr.data(start:stop),mydata1.Fz_fr.data(start:stop),myblue)
plotTire(mydata2.Fy_fr.data(start:stop),mydata2.Fx_fr.data(start:stop),mydata2.Fz_fr.data(start:stop),myred)
hold off

nexttile
plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Black', 'LineWidth', 2);
hold on
plotTire(mydata1.Fy_rl.data(start:stop),mydata1.Fx_rl.data(start:stop),mydata1.Fz_rl.data(start:stop),myblue)
plotTire(mydata2.Fy_rl.data(start:stop),mydata2.Fx_rl.data(start:stop),mydata2.Fz_rl.data(start:stop),myred)
hold off

nexttile
plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Black', 'LineWidth', 2);
hold on
plotTire(mydata1.Fy_rr.data(start:stop),mydata1.Fx_rr.data(start:stop),mydata1.Fz_rr.data(start:stop),myblue)
plotTire(mydata2.Fy_rr.data(start:stop),mydata2.Fx_rr.data(start:stop),mydata2.Fz_rr.data(start:stop),myred)
hold off
axis equal



%% tire utilisation plot
figure(76)

tiledlayout(2,2)
nexttile
x = 1;
vals = [tireUtilisation1_fl; tireUtilisation2_fl];
b1 = bar(x,vals);
legend(label1,label2)
grid on

xtips2 = b1(2).XEndPoints;
ytips2 = b1(2).YEndPoints;
labels2 = string(b1(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
ylim([0.5 1.5])
title("FL tire utilisation")
set(gca,'xtick',[])


nexttile
x = 1;
vals = [tireUtilisation1_fr; tireUtilisation2_fr];
b1 = bar(x,vals);
legend(label1,label2)
grid on

xtips2 = b1(2).XEndPoints;
ytips2 = b1(2).YEndPoints;
labels2 = string(b1(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
ylim([0.5 1.5])
title("FR tire utilisation")
set(gca,'xtick',[])

nexttile
x = 1;
vals = [tireUtilisation1_rl; tireUtilisation2_rl];
b1 = bar(x,vals);
legend(label1,label2)
grid on

xtips2 = b1(2).XEndPoints;
ytips2 = b1(2).YEndPoints;
labels2 = string(b1(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')


xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
ylim([0.5 1.5])
title("RL tire utilisation")
set(gca,'xtick',[])

nexttile
x = 1;
vals = [tireUtilisation1_rr; tireUtilisation2_rr];
b1 = bar(x,vals);
legend(label1,label2)
grid on

xtips2 = b1(2).XEndPoints;
ytips2 = b1(2).YEndPoints;
labels2 = string(b1(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
ylim([0.5 1.5])
title("RR tire utilisation")
set(gca,'xtick',[])
set(gcf, 'Position',  [0, 0, 600*0.8, 400])

Tireutil = sprintf('%s%s_%s%s','renders/','Tireutil',testName,'.eps')
saveas(gcf,Tireutil,'epsc')



%% Example 11: Leave out plotting missing values specified by "Inf".
figure(75)
% Initialize data points
D0 = round([results.points.points2WD.Endurance results.points.points2WD.Skidpad results.points.points2WD.Acceleration results.points.points2WD.Autocross  ]);
D1 = round([results.points.points4WD.Endurance results.points.points4WD.Skidpad results.points.points4WD.Acceleration results.points.points4WD.Autocross  ]);
D2 = round([results.points.points4WDTV.Endurance results.points.points4WDTV.Skidpad results.points.points4WDTV.Acceleration results.points.points4WDTV.Autocross  ]);
D3 = round([results.points.points4WDRS.Endurance results.points.points4WDRS.Skidpad results.points.points4WDRS.Acceleration results.points.points4WDRS.Autocross  ]);
D4 = round([results.points.points4WDgepd.Endurance results.points.points4WDgepd.Skidpad results.points.points4WDgepd.Acceleration results.points.points4WDgepd.Autocross  ]);
P = [ D1; D2; D3;D4];

% Axes limits
axes_limits = [0 0 0 0 ; 250 46.5  46.5 95.5 ];

% Delete variable in workspace if exists
if exist('s1', 'var')
    delete(s1);
end

% Spider plot
t= tiledlayout(1,1);
s1 = spider_plot_class(P);
s1.AxesLimits = axes_limits;
s1.FillOption = 'on';
s1.AxesInterpreter ='latex';
s1.LegendLabels = {'baseline', 'TV', 'RS',"GEPD"};
s1.LegendHandle.Location = 'northeastoutside';
s1.AxesLabels = {'Endurance', 'Skidpad', 'acceleration', 'Autocross'};
s1.AxesInterpreter ='latex';
s1.LabelFontSize=11;
s1.AxesFontSize=11;
s1.LegendHandle.FontSize =11;
s1.LegendHandle.Interpreter = 'latex';
s1.AxesPrecision =0
Points = sprintf('%s%s_%s%s','renders/','Points',testName,'.svg');
set(gcf, 'Position',  [0, 0, 650, 350])
set(gcf,'renderer','Painters')
saveas(gcf,Points,'svg')



%% segment table
time1 = data1.z(stop,6) - data1.z(start,6);
time2 = data2.z(stop,6) - data2.z(start,6);
averageAccel1 = mean(sqrt(mydata1.ax.data(start:stop).^2 + mydata1.ay.data(start:stop).^2));
averageAccel2 = mean(sqrt(mydata2.ax.data(start:stop).^2 + mydata2.ay.data(start:stop).^2));

averageSpeed1 = mean(sqrt(mydata1.vx.data(start:stop).^2 + mydata1.vy.data(start:stop).^2));
averageSpeed2 = mean(sqrt(mydata2.vx.data(start:stop).^2 + mydata2.vy.data(start:stop).^2));

Segment_Values = [label1;label2;"delta"];
Time = [time1;time2;time2-time1];
AverageAcceleration = [averageAccel1;averageAccel2; averageAccel2 - averageAccel1];
AverageSpeed = [averageSpeed1;averageSpeed2 ;averageSpeed2-averageSpeed1];
segmentTable = table(Time,AverageAcceleration,AverageSpeed);
segmentTable.Properties.RowNames = Segment_Values
table2latex(segmentTable,'./renders/comparisonTable.tex')


%% whole endurance
time1 = data1.z(end,6);
time2 = data2.z(end,6) ;
averageAccel1 = mean(sqrt(mydata1.ax.data.^2 + mydata1.ay.data.^2));
averageAccel2 = mean(sqrt(mydata2.ax.data.^2 + mydata2.ay.data.^2));

averageSpeed1 = mean(sqrt(mydata1.vx.data.^2 + mydata1.vy.data.^2));
averageSpeed2 = mean(sqrt(mydata2.vx.data.^2 + mydata2.vy.data.^2));

Segment_Values = [label1;label2;"delta"];
Time = [time1;time2;time2-time1];
AverageAcceleration = [averageAccel1;averageAccel2; averageAccel2 - averageAccel1];
AverageSpeed = [averageSpeed1;averageSpeed2 ;averageSpeed2-averageSpeed1];

wholeTable = table(Time,AverageAcceleration,AverageSpeed);
wholeTable.Properties.RowNames = Segment_Values




%patients= renamevars(patients,["Time","AverageAcceleration"],[label1,label2])
plotTV = 1;

if plotTV
    figure(81)
    plot([mydata1.Fx_fl.data(start:stop), mydata1.Fx_fr.data(start:stop), mydata1.Fx_rl.data(start:stop), mydata1.Fx_rr.data(start:stop)],LineStyle="-",LineWidth = 2)
    hold on
    set(gca,'ColorOrderIndex',1)
    plot([mydata2.Fx_fl.data(start:stop), mydata2.Fx_fr.data(start:stop), mydata2.Fx_rl.data(start:stop), mydata2.Fx_rr.data(start:stop)],LineStyle=":",LineWidth = 2)
    legend("Force FL 4WD","Force FR 4WD","Force RL 4WD","Force RR 4WD", "Force FL 4WDTV","Force FR 4WDTV","Force RL 4WDTV","Force RR 4WDTV",'Interpreter','latex')
    hold off
    xlabel("Position on track [m]",FontSize=11)
    ylabel("Longitudinal Forces [N]",FontSize=11)
    grid on
    set(gcf, 'Position',  [0, 0, 600*0.8, 500*0.5])
    TVrender= append("renders/",label2);
    TVrender= append(TVrender,"TVplot.eps");
    saveas(gcf,"renders/TV.eps",'epsc')

end


if data2.car.steeredAxle == "both"
    figure(82)

    plot(data1.s(start:stop),mydata1.steering_f.data(start:stop),LineWidth=2)
    hold on
    plot(data2.s(start:stop),mydata2.steering_f.data(start:stop),'LineStyle',':',LineWidth=2)
    hold on
    plot(data2.s(start:stop),mydata2.steering_r.data(start:stop),'LineStyle',':',LineWidth=2)
    hold off


    axis tight
    %axis 'auto y'
    %ax100.YLimitMethod = 'padded';
    grid on
    legend("baseline $ \delta_f$","RS $\delta_f$","RS $\delta_r$",'Interpreter','latex','Location','best')
    xlabel("Position on track [m]",FontSize=11)
    ylabel("Steering angle on wheels [deg]",FontSize=11)
    set(gcf, 'Position',  [0, 0, 460, 300])
    pause(1)
    axis 'auto y'

    grid on
    SterringComparison = sprintf('%s%s_%s%s','renders/','SterringComparison',testName,'.eps')
    saveas(gcf,SterringComparison,'epsc')

end





if data2.car.gepdToggle == 1
    figure(84)
    plot(data2.s(start:stop),data2.u(start:stop,7)/1000,'LineWidth',2,LineStyle=':')
    axis tight
    ylim([0 13])
    ylabel("Gepd Power [kW]",'Interpreter','latex')
    xlabel("Position on track [m]",'Interpreter','latex')
    set(gcf, 'Position',  [0, 0, 600*0.8, 500*0.5])
    grid on
    gepd = sprintf('%s%s_%s%s','renders/','gepd',testName,'.eps');
    saveas(gcf,gepd,'epsc')
    legend("GEPD Power [kW]",'Location','best')

end

data4WD       = results.data.data.data4WD(discipline);
data4WDTV     = results.data.data.data4WDTV(discipline);
data4WDRS     = results.data.data.data4WDRS(discipline);
data4WDGEPD   = results.data.data.data4WDgepd(discipline);

mydata4WD     = optdata(data4WD.z,data4WD.u,data4WD.car);
mydata4WDTV   = optdata(data4WDTV.z,data4WDTV.u,data4WDTV.car);
mydata4WDRS   = optdata(data4WDRS.z,data4WDRS.u,data4WDRS.car);
mydata4WDGEPD = optdata(data4WDGEPD.z,data4WDGEPD.u,data4WDGEPD.car);



util4WD     = meanUtilisation(mydata4WD,start,stop)
util4WDTV   = meanUtilisation(mydata4WDTV,start,stop)
util4WDRS   = meanUtilisation(mydata4WDRS,start,stop)
util4WDGEPD = meanUtilisation(mydata4WDGEPD,start,stop)




WD =     [max(sqrt(mydata4WD.ax.data.^2     + mydata4WD.ay.data.^2))      mean(sqrt(mydata4WD.ax.data.^2     + mydata4WD.ay.data.^2))     max(sqrt(mydata4WD.vx.data.^2     + mydata4WD.vy.data.^2))     mean(sqrt(mydata4WD.vx.data.^2     + mydata4WD.vy.data.^2))     util4WD     mydata4WD.t(end)];
WDTV =   [max(sqrt(mydata4WDTV.ax.data.^2   + mydata4WDTV.ay.data.^2))    mean(sqrt(mydata4WDTV.ax.data.^2   + mydata4WDTV.ay.data.^2))   max(sqrt(mydata4WDTV.vx.data.^2   + mydata4WDTV.vy.data.^2))   mean(sqrt(mydata4WDTV.vx.data.^2   + mydata4WDTV.vy.data.^2))   util4WDTV   mydata4WDTV.t(end)];
WDRS =   [max(sqrt(mydata4WDRS.ax.data.^2   + mydata4WDRS.ay.data.^2))    mean(sqrt(mydata4WDRS.ax.data.^2   + mydata4WDRS.ay.data.^2))   max(sqrt(mydata4WDRS.vx.data.^2   + mydata4WDRS.vy.data.^2))   mean(sqrt(mydata4WDRS.vx.data.^2   + mydata4WDRS.vy.data.^2))   util4WDRS   mydata4WDRS.t(end)];
WDGEPD = [max(sqrt(mydata4WDGEPD.ax.data.^2 + mydata4WDGEPD.ay.data.^2))  mean(sqrt(mydata4WDGEPD.ax.data.^2 + mydata4WDGEPD.ay.data.^2)) max(sqrt(mydata4WDGEPD.vx.data.^2 + mydata4WDGEPD.vy.data.^2)) mean(sqrt(mydata4WDGEPD.vx.data.^2 + mydata4WDGEPD.vy.data.^2)) util4WDGEPD mydata4WDGEPD.t(end)];

P = [ WD; WDTV; WDRS; WDGEPD];

% Axes limitsx
axes_limits = [0 0 0 0 ; 250 46.5  46.5 95.5 ];

% Delete variable in workspace if exists
if exist('s', 'var')
    delete(s);
end
figure(85)
% Spider plot
s = spider_plot_class(P);
%s.AxesLimits = axes_limits;
%s.FillOption = 'on';
s.AxesLabels = {'Max acceleration [$m/s^2$]','Mean acceleration [$m/s^2$]', ...
                'Max speed [$m/s$]','Mean speed [$m/s$]','Tire Util.\%', 'Time [$s$]'};
s.LegendLabels = {'baseline', 'TV', 'RS',"GEPD"};
s.FillOption = 'on';
s.AxesInterpreter ='latex';
s.LabelFontSize=11;
s.AxesFontSize=11;
s.LegendHandle.FontSize =11;
s.LegendHandle.Interpreter = 'latex';
s.LegendHandle.Location="best";
set(gcf, 'Position',  [0, 0, 650, 350])
Quantities = sprintf('%s%s_%s%s','renders/','Quantities',testName,'.svg');
saveas(gcf,Quantities,'svg')

s.AxesDirection ={'normal', 'normal', 'normal', 'normal','normal','reverse'};





function util = meanUtilisation(mydata1,start,stop)
tireUtilisation1_fl = mean(tireUtilisation(mydata1.Fx_fl.data(start:stop),mydata1.Fy_fl.data(start:stop),mydata1.Fz_fl.data(start:stop),mydata1.car));
tireUtilisation1_fr = mean(tireUtilisation(mydata1.Fx_fr.data(start:stop),mydata1.Fy_fr.data(start:stop),mydata1.Fz_fr.data(start:stop),mydata1.car));
tireUtilisation1_rl = mean(tireUtilisation(mydata1.Fx_rl.data(start:stop),mydata1.Fy_rl.data(start:stop),mydata1.Fz_rl.data(start:stop),mydata1.car));
tireUtilisation1_rr = mean(tireUtilisation(mydata1.Fx_rr.data(start:stop),mydata1.Fy_rr.data(start:stop),mydata1.Fz_rr.data(start:stop),mydata1.car));

util = mean([tireUtilisation1_fl tireUtilisation1_fr tireUtilisation1_rl tireUtilisation1_rr])*100;


end
function plotTire(x,y,z,farba)

scatter(x./z./1.3, y./z./1.5,15,farba,'filled')
axis equal
hold on
grid on
[k, ~] = convhull([x./z,y./z]);
%plot(x(k)./z(k),y(k)./z(k),"Color",farba,LineWidth=3)
xlabel("Normalised Lateral Force [-]",'Interpreter','latex')
ylabel("Normalised Longitudinal Force [-]",'Interpreter','latex')
end
function tireUtilisation = tireUtilisation(Fx,Fy,Fz,car)


Utilisationx = Fx./(Fz.*car.tire_fxFactor);
Utilisationy = Fy./(Fz.*car.tire_fyFactor);
tireUtilisation = sqrt(Utilisationx.^2 +Utilisationy.^2);



end


