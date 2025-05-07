function  renderVideo(mydata,axes,filename)
%RENDERVIDEO Summary of this function goes here
%   Detailed explanation goes here
%myfig = figure(1)

FPS = 20;

vidObj = VideoWriter(filename);
vidObj.Quality = 95;
            vidObj.FrameRate = FPS;
            open(vidObj);

%[x_track, y_track, th_track, ~] = mydata.track.fcurve(mydata.s);

ax = axes(1) ;
minimap = axes(2);
torques = axes(3);
statistics = axes(4);
fig = axes(5);
cla(ax)

fig.Resize = 'off';

%%
plotTrack(mydata.track,mydata.s,0,ax)

plot1       = scatter(ax,mydata.posCoG(1),mydata.posCoG(1),50,'b','filled');

hold(ax,'on');
axis(ax, 'equal');
grid(ax, 'on');
plot2 = scatter(ax,mydata.posfl(1),mydata.posfl(1),50,'b','filled');
plot3 = scatter(ax,mydata.posfr(1),mydata.posfr(1),50,'b','filled');
plot4 = scatter(ax,mydata.posfr(1),mydata.posfr(1),50,'b','filled');
plot5 = scatter(ax,mydata.posfr(1),mydata.posfr(1),50,'b','filled');

wheelfl = plot(ax,mydata.edgesfl(1:2,1,1)',mydata.edgesfl(1:2,2,1)',LineWidth=3,Color='b');
wheelfr = plot(ax,mydata.edgesfr(1:2,1,1)',mydata.edgesfr(1:2,2,1)',LineWidth=3,Color='b');
wheelrl = plot(ax,mydata.edgesrl(1:2,1,1)',mydata.edgesrl(1:2,2,1)',LineWidth=3,Color='b');
wheelrr = plot(ax,mydata.edgesrr(1:2,1,1)',mydata.edgesrr(1:2,2,1)',LineWidth=3,Color='b');
boxSize = 5;


%render_time = 30*
t_visu = mydata.t(1):(1/FPS):mydata.t(end);

interp1(mydata.t(1:end-1), mydata.steering_f.data, t_visu)





interp1(mydata.t(1:end-1), mydata.steering_f.data, t_visu)

%% interpolate CoG
cogX  = interp1(mydata.t(1:end-1), squeeze(mydata.posCoG(:,1,1:end-1)), t_visu);
cogY  = interp1(mydata.t(1:end-1), squeeze(mydata.posCoG(:,2,1:end-1)), t_visu);
%% interpolate positions
posflx = interp1(mydata.t(1:end-1), squeeze(mydata.posfl(:,1,1:end-1)), t_visu);
posfly = interp1(mydata.t(1:end-1), squeeze(mydata.posfl(:,2,1:end-1)), t_visu);

posfrx = interp1(mydata.t(1:end-1), squeeze(mydata.posfr(:,1,1:end-1)), t_visu);
posfry = interp1(mydata.t(1:end-1), squeeze(mydata.posfr(:,2,1:end-1)), t_visu);

posrlx = interp1(mydata.t(1:end-1), squeeze(mydata.posrl(:,1,1:end-1)), t_visu);
posrly = interp1(mydata.t(1:end-1), squeeze(mydata.posrl(:,2,1:end-1)), t_visu);

posrrx = interp1(mydata.t(1:end-1), squeeze(mydata.posrr(:,1,1:end-1)), t_visu);
posrry = interp1(mydata.t(1:end-1), squeeze(mydata.posrr(:,2,1:end-1)), t_visu);

%% interpolate wheels
wheelflx = interp1(mydata.t(1:end-1), squeeze(mydata.edgesfl(1:2,1,1:end-1))', t_visu);
wheelfly = interp1(mydata.t(1:end-1), squeeze(mydata.edgesfl(1:2,2,1:end-1))', t_visu);

wheelfrx = interp1(mydata.t(1:end-1), squeeze(mydata.edgesfr(1:2,1,1:end-1))', t_visu);
wheelfry = interp1(mydata.t(1:end-1), squeeze(mydata.edgesfr(1:2,2,1:end-1))', t_visu);

wheelrlx = interp1(mydata.t(1:end-1), squeeze(mydata.edgesrl(1:2,1,1:end-1))', t_visu);
wheelrly = interp1(mydata.t(1:end-1), squeeze(mydata.edgesrl(1:2,2,1:end-1))', t_visu);

wheelrrx = interp1(mydata.t(1:end-1), squeeze(mydata.edgesrr(1:2,1,1:end-1))', t_visu);
wheelrry = interp1(mydata.t(1:end-1), squeeze(mydata.edgesrr(1:2,2,1:end-1))', t_visu);

%% interpolate torques
flT  = interp1(mydata.t(1:end-1), mydata.Fy_fl.data(1:end)', t_visu);
frT  = interp1(mydata.t(1:end-1), mydata.Fy_fr.data(1:end)', t_visu);
rlT  = interp1(mydata.t(1:end-1), mydata.Fy_rl.data(1:end)', t_visu);
rrT  = interp1(mydata.t(1:end-1), mydata.Fy_rr.data(1:end)', t_visu);


%% interpolate speed

vx = interp1(mydata.t(1:end-1), mydata.expdata.z(1:end-1,1)', t_visu);



%% write create rectangels
width = 10;
cla(torques)
fl = rectangle(torques, 'Position', [ -5, 10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
fr = rectangle(torques, 'Position', [ 5, 10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');


rl = rectangle(torques, 'Position', [ -5, -10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
rr = rectangle(torques, 'Position', [ 5, -10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');


xlim(torques,[-5 15])
ylim(torques,[-20 20])

%% Minimap
cla(minimap)
plotTrack(mydata.track,mydata.s,0,minimap)
hold(minimap ,'on')
axis(minimap,'equal')
grid(minimap,'on')
CoGminimap  = scatter(minimap,mydata.posCoG(1),mydata.posCoG(1),50,'b','filled');

%% torques


for i = 1:numel(t_visu)-2%length(mydata.s)-1
    %% Main plot
        plot1.XData = cogX(i);
    plot1.YData = cogY(i);

    plot2.XData = posflx(i);
    plot2.YData = posfly(i);

    plot3.XData = posfrx(i);
    plot3.YData = posfry(i);

    plot4.XData = posrlx(i);
    plot4.YData = posrly(i);

    plot5.XData = posrrx(i);
    plot5.YData = posrry(i);


    wheelfl.XData = wheelflx(i,:);
    wheelfl.YData = wheelfly(i,:);

    wheelfr.XData = wheelfrx(i,:);
    wheelfr.YData = wheelfry(i,:);

    wheelrl.XData = wheelrlx(i,:);
    wheelrl.YData = wheelrly(i,:);

    wheelrr.XData = wheelrrx(i,:);
    wheelrr.YData = wheelrry(i,:);

    xlim(ax,[cogX(i)-boxSize cogX(i)+boxSize])
    ylim(ax,[cogY(i)-boxSize cogY(i)+boxSize])

    drawnow
    grid(ax,'on')
    %% Minimap
    CoGminimap.XData = cogX(i);
    CoGminimap.YData = cogY(i);

    %% torques
    torquePlot(flT(i),fl,0)
    torquePlot(frT(i),fr,0)
    torquePlot(rlT(i),rl,1)
    torquePlot(rrT(i),rr,1)

     %fr.Position(4) =  abs(frT(i)/162.55);
     %rl.Position(4) =  abs(rlT(i)/162.55);
     %rr.Position(4) =  abs(rrT(i)/162.55);

     %% statistics
     cla(statistics)
     text(statistics, 0.05, 0.7, sprintf('Speed: %.1f km/h', vx(i)*3.6), 'FontSize', 14);
    text(statistics, 0.05, 0.5, sprintf('Time: %.1f s', t_visu(i)), 'FontSize', 14);
    
    writeVideo(vidObj, getframe(fig));

end

close(vidObj)
fig.Resize = 'on';
end

function torquePlot(torque, rect,pos)
    if torque > 0 && pos == 0
        rect.Position(4) =  torque/162.55;
        rect.Position(2) =  10;
        rect.FaceColor = [0 1 0];
    end

    if torque < 0 && pos == 0
        rect.Position(2) =  10 + torque/162.55;
        rect.Position(4) = -torque/162.55;
        rect.FaceColor = [1 0 0];
        return
    end

    if torque > 0 && pos == 1
        rect.Position(4) =  torque/162.55;
        rect.Position(2) =  10-20;
        rect.FaceColor = [0 1 0];
    end
    if torque < 0 && pos == 1
        rect.Position(2) =  10 + torque/162.55-20;
        rect.Position(4) = -torque/162.55;
        rect.FaceColor = [1 0 0];
    end


end


%fl = rectangle(torques, 'Position', [ -5, 10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
%fr = rectangle(torques, 'Position', [ 5, 10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');


%rl = rectangle(torques, 'Position', [ -5, -10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
%rr = rectangle(torques, 'Position', [ 5, -10, width, width],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');

