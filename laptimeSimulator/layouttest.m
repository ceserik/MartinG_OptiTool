myfig = figure(1)

[lol mydata]=twinRaceCar_path_ODE(expdata.s',expdata.z',expdata.u',car,expdata.track,expdata)   
tiledlayout(1,1);
first = nexttile;
plot1  = scatter(first,mydata.posCoG(1),mydata.posCoG(1),50,'filled');
hold on
axis equal
grid on

[x_track, y_track, th_track, ~] = expdata.track.fcurve(expdata.s);


wheel_length = 0.5;

plot2 = scatter(first,mydata.posfl(1),mydata.posfl(1),50,'filled');
plot3 = scatter(first,mydata.posfr(1),mydata.posfr(1),50,'filled');
plot4 = scatter(first,mydata.posfr(1),mydata.posfr(1),50,'filled');
plot5 = scatter(first,mydata.posfr(1),mydata.posfr(1),50,'filled');


%set(gca, 'XDir', 'reverse')
wheelData = [mydata.edgesfl(1,1:2,1)' mydata.edgesfl(2,1:2,1)']

plotTrack(expdata.track,expdata.s)

wheelfl = plot(mydata.edgesfl(1:2,1,1)',mydata.edgesfl(1:2,2,1)');
wheelfr = plot(mydata.edgesfr(1:2,1,1)',mydata.edgesfr(1:2,2,1)');
wheelrl = plot(mydata.edgesrl(1:2,1,1)',mydata.edgesrl(1:2,2,1)');
wheelrr = plot(mydata.edgesrr(1:2,1,1)',mydata.edgesrr(1:2,2,1)');
%fl_pos = mydata.
%second= nexttile;

boxSize = 5;


for i = 1:length(mydata.s)-1
    plot1.XData = squeeze(mydata.posCoG(:,1,i));
    plot1.YData = squeeze(mydata.posCoG(:,2,i));
    
    plot2.XData = squeeze(mydata.posfl(:,1,i));
    plot2.YData = squeeze(mydata.posfl(:,2,i));

    plot3.XData = squeeze(mydata.posfr(:,1,i));
    plot3.YData = squeeze(mydata.posfr(:,2,i));

    plot4.XData = squeeze(mydata.posrl(:,1,i));
    plot4.YData = squeeze(mydata.posrl(:,2,i));

    plot5.XData = squeeze(mydata.posrr(:,1,i));
    plot5.YData = squeeze(mydata.posrr(:,2,i));


    wheelfl.XData = mydata.edgesfl(1:2,1,i)';
    wheelfl.YData = mydata.edgesfl(1:2,2,i)';
    
    wheelfr.XData = mydata.edgesfr(1:2,1,i)';
    wheelfr.YData = mydata.edgesfr(1:2,2,i)';

    wheelrl.XData = mydata.edgesrl(1:2,1,i)';
    wheelrl.YData = mydata.edgesrl(1:2,2,i)';

    wheelrr.XData = mydata.edgesrr(1:2,1,i)';
    wheelrr.YData = mydata.edgesrr(1:2,2,i)';



    xlim([squeeze(mydata.posCoG(:,1,i))-boxSize squeeze(mydata.posCoG(:,1,i))+boxSize])
    ylim([squeeze(mydata.posCoG(:,2,i))-boxSize squeeze(mydata.posCoG(:,2,i))+boxSize])

    drawnow
    %pause(0.01)
    
    grid on
end

