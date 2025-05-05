% Create UIFigure
%fig = uifigure('Name', 'Custom Layout',[0 0 1600 900],'Resize', 'off');
import casadi.*
%profile clear;
%profile on;
addpath('/home/Riso/Downloads/casadimojemoje/')
fig = uifigure('Name', 'My App', 'Position', [0 0 1600 900], 'Resize', 'off');


% Main grid: 1 row x 2 columns
mainGrid = uigridlayout(fig, [1, 2]);
mainGrid.ColumnWidth = {'2x','1x'};  % Left big plot, right narrow
mainGrid.RowHeight = {'1x'};

% === Left: Big Plot ===
axMain = uiaxes(mainGrid);
title(axMain, 'Main Plot');
plot(axMain, sin(0:0.1:10));

% === Right: Sub-grid with 2 rows ===
rightGrid = uigridlayout(mainGrid, [4, 1]);
rightGrid.RowHeight = {'1x', '1x','1x','1x'};  % Top plot flexible, bottom controls
rightGrid.ColumnWidth = {'1x'};

% === Top Right: Small Plot ===
minimap = uiaxes(rightGrid);
title(minimap, 'Minimap');
%plot(minimap, rand(10,1));

% === Top Right: Small Plot ===
torques = uiaxes(rightGrid);
title(torques, 'Torques');
%plot(torques, rand(10,1));

% === Top Right: Small Plot ===
statistics = uiaxes(rightGrid);
title(statistics, 'Statistics');
plot(statistics, rand(10,1));
% Turn off axes visuals
statistics.XColor = 'none';
statistics.YColor = 'none';
statistics.XTick = [];
statistics.YTick = [];
axis(statistics, [0 1 0 1]);  % Normalized axis for placing text easily
title(statistics, 'Statistics');

% Example speed and time values
speed = 0;      % km/h
elapsedTime = 0;  % seconds

% Display values using text
text(statistics, 0.05, 0.7, sprintf('Speed: %.1f km/h', speed), 'FontSize', 14);
text(statistics, 0.05, 0.5, sprintf('Time: %.1f s', elapsedTime), 'FontSize', 14);



% === Bottom Right: Controls ===
controlPanel = uipanel(rightGrid);
controlPanel.Title = 'Controls';

% Add buttons inside control panel
controlGrid = uigridlayout(controlPanel, [4, 1]);
btnRender = uibutton(controlGrid, 'Text', 'Render');

btnSimulate = uibutton(controlGrid, 'Text', 'Simulate');
filename = char(string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".avi");
filenameInput = uieditfield(controlGrid, 'text', ...
    'Placeholder', 'e.g. data_run_01.csv', ...
    'Value', filename);



%powertrainType = char(string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".avi");
powertrain = uieditfield(controlGrid, 'text', ...
    'Placeholder', 'e.g. data_run_01.csv', ...
    'Value', "4WD");



%% torques



btnRender.ButtonPushedFcn = @(src, event) render([axMain minimap torques statistics,fig],fig.UserData.mydata,filenameInput);

btnSimulate.ButtonPushedFcn = @(src, event) simulate(fig,powertrain);

function render(axes,mydata,filename)
    if ~endsWith(filename.Value, '.avi', 'IgnoreCase', true)
        warning('Filename must end with ".avi"');
        return;
end

renderVideo(mydata,axes,filename.Value)
end




function[time,expdata,opti,sol,mydata]  =  simulate(fig,powertrain)
if string(powertrain.Value) == "4WD" || string(powertrain.Value) == "4WDTV" || string(powertrain.Value) == "2WD"

    car = carCreate("HAFO24",0,powertrain.Value);

    [time,expdata,opti,sol,mydata] = runLaptime(car,"doubleTurn");
    fig.UserData.mydata = mydata;
    fig.UserData.expdata = expdata;
else
    warning("wrong powertrain")

    return
end


end





