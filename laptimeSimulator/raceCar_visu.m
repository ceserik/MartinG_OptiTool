function raceCar_visu(exp, varargin)
% exp - cell of structures containing the following fields:
% 			s - arc length along the curve
% 			z = [vx; vy; psi; dpsidt; n; t],
%                vx-longitudinal velocity,
%                vy-lateral velocity,
%                psi-angular position of the car with respect to the inertial frame,
%                n-perpendicular distance to the curve,
%                t-time
% 			u = [delta_f; delta_r; Ff; Fr],
%				 delta_f - steering angle of the front wheel,
%				 delta_r - steering angle of the rear wheel,
%				 Ff - force acting on the front wheel in the longitudinal direction,
%				 Fr - force acting on the rear wheel in the longitudinal direction
% 	   carpos = [x, y] - N-by-2 matrix containing position of the car
% 		  car - structure containing parameters of the car model
% 		track - structure containing:
% 			         [xc, yc, th, C] = fcurve(s) - th=angle of the curve, C=curviture of the curve
% 			         width - width of the track in meters
% Additional parameters:
%			'FileName' - if provided, an animation is stored to a file with this
%						 file name. Currently, only .gif and .avi are supported
%			     'FPS' - frame rate of the visualization
%	'ColoredTrackLine' - the track line behind the car has varying color dependent
%                        on the speed of the car
%	  'ShowTireForces' - show the force acting on individual tires
%	'TireForceMaxSize' - the length (in meters) of the line showing the maximum
%                        force acting on a tire
%'MagnifiedWindowSize' - size (in meters) of the window showing a cut-out of the
%                        track with the car in the center


% Define and parse input arguments
p = inputParser;
addParameter(p, 'FileName',            [],     @ischar);
addParameter(p, 'FPS',                 60,     @isnumeric);
addParameter(p, 'ColoredTrackLine',    false,  @islogical);
addParameter(p, 'ShowTireForces',      false,  @islogical);
addParameter(p, 'TireForceMaxSize',    0.25,    @isnumeric);
addParameter(p, 'MagnifiedWindowSize', 5,     @isnumeric);
parse(p, varargin{:});

if ~iscell(exp)
    exp = {exp};
end

N_exps = numel(exp); % number of experiments to be visualized

for i=1:N_exps
    % Check whether the 'exp{i}' structure contains all needed data
    if isstruct(exp{i})
        if isfield(exp{i}, 's') && isfield(exp{i}, 'z') && ...
                isfield(exp{i}, 'u') && isfield(exp{i}, 'car') && ...
                isfield(exp{i}, 'carpos') && isfield(exp{i}, 'track') &&...
                isfield(exp{i}, 'optparams')

            s{i}         = exp{i}.s;vidObj
            z{i}         = exp{i}.z;
            u{i}         = exp{i}.u;
            carpos{i}    = exp{i}.carpos;
            car{i}       = exp{i}.car;
            track        = exp{i}.track; % we assume that all experiments were carried out on the same track
            optparams{i} = exp{i}.optparams;
        else
            error('The provided structure does not containt all data from an experiment.')
        end
    else
        error('You must provide the function with a structure containing all data from the experiment.')
    end
end

% If FileName is provided, initialize the
if ~isempty(p.Results.FileName)
    [~,~, ext] = fileparts(p.Results.FileName);

    switch lower(ext)
        case '.gif'
        case '.avi'
            vidObj = VideoWriter(p.Results.FileName);
            vidObj.Quality = 95;
            vidObj.FrameRate = p.Results.FPS;
            open(vidObj);
        otherwise
            error('This file extension is not supported for storing the animation file.')
    end

end

h = figure(25);
clf

% First subplot
h_sp1 = subplot(3, 3, [1 2 4 5 7 8]);
clrs = h_sp1.ColorOrder;
hold on

for i=1:N_exps
    % Interpolate the states so that we get the state values for each frame
    t{i}      = z{i}(:,6);
    t_visu{i} = t{i}(1):(1/p.Results.FPS):t{i}(end);
    s_visu{i} = interp1(t{i}, s{i}, t_visu{i});
    z_visu{i} = interp1(t{i}, z{i}, t_visu{i});
    u_visu{i} = interp1(t{i}(1:end-1), u{i}, t_visu{i});

    % Get the position and heading of the track
    [x_track, y_track, th_track, ~] = track.fcurve(s{i});

    % Extract the needed states
    vx_car{i}   = z_visu{i}(:,1);
    vy_car{i}   = z_visu{i}(:,2);
    psi_car{i}  = z_visu{i}(:,3);
    dpsi_car{i} = z_visu{i}(:,4);
    n_opt{i}    = z_visu{i}(:,5);
    Fz{i}       = z_visu{i}(:,7);
    Ltl{i}      = z_visu{i}(:,8);
    x_track_sampledInTime{i}  = interp1(s{i}, x_track, s_visu{i});
    y_track_sampledInTime{i}  = interp1(s{i}, y_track, s_visu{i});
    th_track_sampledInTime{i} = interp1(s{i}, th_track, s_visu{i});

    x_car{i}   = -n_opt{i}.*sin(th_track_sampledInTime{i}') + x_track_sampledInTime{i}';
    y_car{i}   =  n_opt{i}.*cos(th_track_sampledInTime{i}') + y_track_sampledInTime{i}';
    v_car{i}   =  sqrt(vx_car{i}.^2 + vy_car{i}.^2);

    % Extract the needed control
    
   
    if car{i}.tracks == 2
        


        delta_f{i}    = u_visu{i}(:,1);
        delta_r{i}    = u_visu{i}(:,2);
        Ffl_x{i}      = u_visu{i}(:,3) + u_visu{i}(:,8)*car{i}.BrakeBalance/2;
        Ffr_x{i}      = u_visu{i}(:,4) + u_visu{i}(:,8)/2*car{i}.BrakeBalance;
        Frl_x{i}      = u_visu{i}(:,5) + u_visu{i}(:,8)/2*(1-car{i}.BrakeBalance);
        Frr_x{i}      = u_visu{i}(:,6) +u_visu{i}(:,8)/2*(1-car{i}.BrakeBalance);
        gepdPower{i} = u_visu{i}(:,7);
        cnt = 1;
        gepdForce = gepdPower{i}/car{i}.gepdMaxPower .*(car{i}.gepdForceCoeff(1) + vx_car{i}.*car{i}.gepdForceCoeff(2));
        Fz_aero{i} = 0.5*car{i}.airDensity*car{i}.CL*car{i}.A*vx_car{i}.^2 - gepdForce;

        Fz_fl{i} = car{i}.m/2*(1-car{i}.COGr) * car{i}.g - Fz{i}/2- Fz_aero{i} * (1-car{i}.COP) - Ltl{i}/2 ;
        Fz_fr{i} = car{i}.m/2*(1-car{i}.COGr) * car{i}.g - Fz{i}/2- Fz_aero{i} * (1-car{i}.COP) + Ltl{i}/2  ;
        Fz_rl{i} = car{i}.m/2*(car{i}.COGr) * car{i}.g + Fz{i}/2- Fz_aero{i} * (car{i}.COP) - Ltl{i}/2     ;
        Fz_rr{i} = car{i}.m/2*(car{i}.COGr) * car{i}.g + Fz{i}/2- Fz_aero{i} * (car{i}.COP) + Ltl{i}/2      ;
         
    Fz_f{i} = car{i}.m/2*(1-car{i}.COGr) * car{i}.g - Fz{i}- Fz_aero{i} * (1-car{i}.COP);
   Fz_r{i} = car{i}.m/2*(car{i}.COGr) * car{i}.g + Fz{i}- Fz_aero{i} * (car{i}.COP);



        p_fl{i} = [car{i}.l_f*cos(psi_car{i}(cnt))  - car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt),  car{i}.l_f*sin(psi_car{i}(cnt)) + car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
        p_rl{i} = [-car{i}.l_r*cos(psi_car{i}(cnt)) - car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), -car{i}.l_r*sin(psi_car{i}(cnt)) + car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
        p_fr{i} = [car{i}.l_f*cos(psi_car{i}(cnt))  + car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt),  car{i}.l_f*sin(psi_car{i}(cnt)) - car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
        p_rr{i} = [-car{i}.l_r*cos(psi_car{i}(cnt)) + car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), -car{i}.l_r*sin(psi_car{i}(cnt)) - car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];

        p_aerofl{i} = [car{i}.l_f*cos(psi_car{i}(cnt))  - car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt),  (car{i}.l_f + car{i}.wingLength_f)*sin(psi_car{i}(cnt)) + car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
        p_aerofr{i} = [car{i}.l_f*cos(psi_car{i}(cnt))  + car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt),  (car{i}.l_f + car{i}.wingLength_f)*sin(psi_car{i}(cnt)) - car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
        
        [lol carVars]=twinRaceCar_path_ODE(s_visu{i}',z_visu{i}',u_visu{i}',car{i},track);
        %carVars = twinTransformations(z_visu{i},u_visu{i},car{i});
        
        alpha_fl{i} = carVars(:,17);
        alpha_fr{i} = carVars(:,18);
        alpha_rl{i} = carVars(:,19);
        alpha_rr{i} = carVars(:,20);

        Ffl_y{i} = car{i}.ftire(alpha_fl{i},Fz_fl{i},car{i});
        Ffr_y{i} = car{i}.ftire(alpha_fr{i},Fz_fr{i},car{i});
        Frl_y{i} = car{i}.ftire(alpha_rl{i},Fz_rl{i},car{i});
        Frr_y{i} = car{i}.ftire(alpha_rr{i},Fz_rr{i},car{i});

    else
        delta_f{i}   = u_visu{i}(:,1);
        delta_r{i}   = u_visu{i}(:,2);
        Ff_x{i}      = u_visu{i}(:,3);
        Fr_x{i}      = u_visu{i}(:,4);


        Ff_y{i} = car{i}.ftire(alpha_f{i},Fz_f{i},car{i});
        Fr_y{i} = car{i}.ftire(alpha_r{i},Fz_r{i},car{i});
    end
    % Compute the tire forces in y direction (in tire coordinates)


    alpha_f{i} = delta_f{i} - atan((vy_car{i} + car{i}.l_f.*dpsi_car{i}) ./ vx_car{i});
    alpha_r{i} = delta_r{i} - atan((vy_car{i} - car{i}.l_r.*dpsi_car{i}) ./ vx_car{i});


    Ff_y{i} = car{i}.ftire(alpha_f{i},Fz_f{i},car{i});
    Fr_y{i} = car{i}.ftire(alpha_r{i},Fz_r{i},car{i});
    
    ftire_max_fl{i} = Fz_fl{i}*car{i}.tire_fyFactor;%car{i}.ftire(optparams{i}.alpha_sat,655*2);
    ftire_max_fr{i} = Fz_fr{i}*car{i}.tire_fyFactor;%car{i}.ftire(optparams{i}.alpha_sat,655*2);
    ftire_max_rl{i} = Fz_rl{i}*car{i}.tire_fyFactor;%car{i}.ftire(optparams{i}.alpha_sat,655*2);
    ftire_max_rr{i} = Fz_rr{i}*car{i}.tire_fyFactor;%car{i}.ftire(optparams{i}.alpha_sat,655*2);
    
    ftire_static{i} = car{i}.m/4*car{i}.g*car{i}.tire_fyFactor;

    % Visualization parameters
    car_clr{i}       = clrs(i, :);
    traj_clr{i}       = clrs(i, :);

    p_f{i} = [car{i}.l_f*cos(psi_car{i}(1)) + x_car{i}(1), car{i}.l_f*sin(psi_car{i}(1)) + y_car{i}(1)];
    p_r{i} = [-car{i}.l_r*cos(psi_car{i}(1)) + x_car{i}(1), -car{i}.l_r*sin(psi_car{i}(1)) + y_car{i}(1)];

end

wheel_clr     = [1 0 1];
tireForce_clr = [0 0 1];

% Plot the track
plotTrack(track, s{end});

max_speed = 0; % used for maximum value of the color bar showing the velocity of the cars
for i=1:N_exps
    % Plot the car and its trajecto
    if p.Results.ColoredTrackLine
        h_trackLine{i} = surface([x_car{i}';x_car{i}'],[y_car{i}';y_car{i}'],[zeros(2,numel(x_car{i}))],[v_car{i}';v_car{i}']*3.6,...
            'facecol','no',...
            'edgecol','interp',...
            'linew',2);

        if max(v_car{i}*3.6) > max_speed
            max_speed = max(v_car{i}*3.6);
        end
    else
        h_trackLine{i}  = plot(0,0,'-', 'LineWidth', 1.5, 'Color', traj_clr{i});
    end

    h_car_COM{i}  = plot(x_car{i}(1), y_car{i}(1), 'o', 'Color', car_clr{i});
    h_car_body{i} = plot([p_f{i}(1) p_r{i}(1)], [p_f{i}(2) p_r{i}(2)], ...
        'LineWidth', 2, ...
        'Color', car_clr{i});
    
end

if p.Results.ColoredTrackLine
    c = colorbar;
    c.Label.String = 'Speed [km/h]';
    caxis([0 max_speed])
    colormap jet
end

hold off

axis equal;

x_track_min = min(x_track) - min(track.w_l, track.w_r) - 1;
x_track_max = max(x_track) + max(track.w_l, track.w_r) + 1;
y_track_min = min(y_track) - min(track.w_l, track.w_r) - 1;
y_track_max = max(y_track) + max(track.w_l, track.w_r) + 1;

axis([x_track_min, x_track_max, y_track_min, y_track_max]);

xlabel('x [m]')
ylabel('y [m]')

% Second subplot
h_sp2 = subplot(3, 3, [3 6]);
hold on
% Plot the track
plotTrack(track, s{end});

for i=1:N_exps
    % Plot the car and its trajectory
    h_traj2{i}  = plot(0,0,'-', 'LineWidth', 1, 'Color', traj_clr{i});  % trajektorie
    h_car_COM2{i}  = plot(x_car{i}(1), y_car{i}(1), 'o', 'Color', car_clr{i});
    h_car_body2{i} = plot([p_f{i}(1) p_r{i}(1)], [p_f{i}(2) p_r{i}(2)], ...
        'LineWidth', 2, ...
        'Color', car_clr{i});
    if car{i}.tracks ==2
        h_car_wing_f{i} = plot([p_aerofr{i}(1) p_aerofl{i}(1)] ,[p_aerofr{i}(2) p_aerofl{i}(2)],'LineWidth', 2, 'Color', car_clr{i});

        h_car_Wheel_fl{i} = plot(car{i}.wheelradius*[cos(psi_car{i}(1) + delta_f{i}(1)) -cos(psi_car{i}(1) + delta_f{i}(1))] + p_fl{i}(1), ...
            car{i}.wheelradius*[sin(psi_car{i}(1) + delta_f{i}(1)) -sin(psi_car{i}(1) + delta_f{i}(1))] + p_fl{i}(2), ...
            'LineWidth', 2, 'Color', wheel_clr);

        h_car_Wheel_fr{i} = plot(car{i}.wheelradius*[cos(psi_car{i}(1) + delta_f{i}(1)) -cos(psi_car{i}(1) + delta_f{i}(1))] + p_fr{i}(1), ...
            car{i}.wheelradius*[sin(psi_car{i}(1) + delta_f{i}(1)) -sin(psi_car{i}(1) + delta_f{i}(1))] + p_fr{i}(2), ...
            'LineWidth', 2, 'Color', wheel_clr);

        h_car_Wheel_rl{i} = plot(car{i}.wheelradius*[cos(psi_car{i}(1) + delta_r{i}(1)) -cos(psi_car{i}(1) + delta_r{i}(1))] + p_rl{i}(1), ...
            car{i}.wheelradius*[sin(psi_car{i}(1) + delta_r{i}(1)) -sin(psi_car{i}(1) + delta_r{i}(1))] + p_rl{i}(2), ...
            'LineWidth', 2, 'Color', wheel_clr);

        h_car_Wheel_rr{i} = plot(car{i}.wheelradius*[cos(psi_car{i}(1) + delta_r{i}(1)) -cos(psi_car{i}(1) + delta_r{i}(1))] + p_rr{i}(1), ...
            car{i}.wheelradius*[sin(psi_car{i}(1) + delta_r{i}(1)) -sin(psi_car{i}(1) + delta_r{i}(1))] + p_rr{i}(2), ...
            'LineWidth', 2, 'Color', wheel_clr);
    else
        h_car_frontwheel2{i} = plot(car{i}.wheelradius*[cos(psi_car{i}(1) + delta_f{i}(1)) -cos(psi_car{i}(1) + delta_f{i}(1))] + p_f{i}(1), ...
            car{i}.wheelradius*[sin(psi_car{i}(1) + delta_f{i}(1)) -sin(psi_car{i}(1) + delta_f{i}(1))] + p_f{i}(2), ...
            'LineWidth', 2, 'Color', wheel_clr);

        h_car_rearwheel2{i} = plot(car{i}.wheelradius*[cos(psi_car{i}(1) + delta_r{i}(1)) -cos(psi_car{i}(1) + delta_r{i}(1))] + p_r{i}(1), ...
            car{i}.wheelradius*[sin(psi_car{i}(1) + delta_r{i}(1)) -sin(psi_car{i}(1) + delta_r{i}(1))] + p_r{i}(2), ...
            'LineWidth', 2, 'Color', wheel_clr);
    end

    r = psi_car{i}(1) + delta_r{i}(1);
    rotation = [ cos(r)  -sin(r); sin(r)   cos(r)];
    maxForceCircle = p.Results.TireForceMaxSize.*[1 * cos(linspace(0, 2*pi, 100)); car{i}.tire_fxFactor/car{i}.tire_fyFactor*sin(linspace(0, 2*pi, 100))] ;
    staticforceCircle = p.Results.TireForceMaxSize.*[1 * cos(linspace(0, 2*pi, 100)); car{i}.tire_fxFactor/car{i}.tire_fyFactor*sin(linspace(0, 2*pi, 100))] ;

    if car{i}.tracks == 2
        h_Ffl{i} = plot(0, 0, 'Color', tireForce_clr);
        h_Ffr{i} = plot(0, 0, 'Color', tireForce_clr);
        h_Frl{i} = plot(0, 0, 'Color', tireForce_clr);
        h_Frr{i} = plot(0, 0, 'Color', tireForce_clr);

        % Tire forces
        if p.Results.ShowTireForces

            h_Ffl_max_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Green', 'LineWidth', 1);
            h_Ffr_max_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Green', 'LineWidth', 1);

            h_Frl_max_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Green', 'LineWidth', 1);
            h_Frr_max_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Green', 'LineWidth', 1);

            h_Ffl_static_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 0.7*[1 1 1], 'LineWidth', 0.5);
            h_Ffr_static_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 0.7*[1 1 1], 'LineWidth', 0.5);
            h_Frl_static_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 0.7*[1 1 1], 'LineWidth', 0.5);
            h_Frr_static_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 0.7*[1 1 1], 'LineWidth', 0.5);
        end
    else
        h_Ff{i} = plot(0, 0, 'Color', tireForce_clr);
        h_Fr{i} = plot(0, 0, 'Color', tireForce_clr);

        % Tire forces
        if p.Results.ShowTireForces
            r = psi_car{i}(1) + delta_r{i}(1);
            rotation = [ cos(r)  -sin(r); sin(r)   cos(r)];
            maxForceCircle = p.Results.TireForceMaxSize.*[1 * cos(linspace(0, 2*pi, 20)); car{i}.tire_fxFactor/car{i}.tire_fyFactor*sin(linspace(0, 2*pi, 20))] ;
            staticforceCircle = p.Results.TireForceMaxSize.*[1 * cos(linspace(0, 2*pi, 20)); car{i}.tire_fxFactor/car{i}.tire_fyFactor*sin(linspace(0, 2*pi, 20))] ;

            h_Ff_max_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Green', 'LineWidth', 1);
            h_Fr_max_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 'Green', 'LineWidth', 1);

            h_Ff_static_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 0.7*[1 1 1], 'LineWidth', 0.5);
            h_Fr_static_circle{i} = plot(maxForceCircle(1, :), maxForceCircle(2, :), 'Color', 0.7*[1 1 1], 'LineWidth', 0.5);

        end
    end


    hold off
    axis equal;

    h_sp2.XTick = [];
    h_sp2.YTick = [];

    box on;

    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset;
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];


    % Third subplot
    h_sp3 = subplot(3, 3, 9);
    cla

    hold on

    % Time
    h_time = text(0.05, 0.5-0.05, '00.00', ...
        'FontSize', 20, ...
        'FontUnits', 'normalized', ...
        'VerticalAlignment', 'top');
    text(0.85, 0.5-0.05, 's', ...
        'FontSize', 15, ...
        'FontUnits', 'normalized', ...
        'VerticalAlignment', 'top');

    if N_exps == 1
        % Speedometer
        h_speedometer = text(0.05, 1-0.05, '100', ...
            'FontSize', 20, ...
            'FontUnits', 'normalized', ...
            'VerticalAlignment', 'top');
        text(0.55, 1-0.05, 'km/h', ...
            'FontSize', 15, ...
            'FontUnits', 'normalized', ...
            'VerticalAlignment', 'top');

        % Forces
        F_y0 = 0.5;
        F_xs = 1.1;
        F_xe = 1.9;
        F_xw = F_xe - F_xs;
        %plot([F_xs F_xe], F_y0*[1 1], 'k')

        %h_Fflx_bar = fill([F_xw/10, F_xw/4+F_xw/10, F_xw/4+F_xw/10, F_xw/10] + F_xs + F_xw/2, [F_y0, F_y0, 0.9, 0.9], [1 0 0], 'LineStyle', 'none');
        %h_Ffrx_bar = fill([F_xw/10, F_xw/4+F_xw/10, F_xw/4+F_xw/10, F_xw/10] + F_xs, [F_y0, F_y0, 0.9, 0.9], [1 0 0], 'LineStyle', 'none');
        ggv = plot(0);
    end

    hold off
    axis equal;
    axis([0 2 0 1])

    h_sp3.XTick = [];
    h_sp3.YTick = [];

    box on;



    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset;
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];



    % vizualizace
    for cnt = 1:numel(t_visu{1})
        tic

        for i=1:N_exps
            p_f = [car{i}.l_f*cos(psi_car{i}(cnt)) + x_car{i}(cnt), car{i}.l_f*sin(psi_car{i}(cnt)) + y_car{i}(cnt)];
            p_r = [-car{i}.l_r*cos(psi_car{i}(cnt)) + x_car{i}(cnt), -car{i}.l_r*sin(psi_car{i}(cnt)) + y_car{i}(cnt)];

            p_fl = [car{i}.l_f*cos(psi_car{i}(cnt))  - car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), car{i}.l_f*sin(psi_car{i}(cnt)) + car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
            p_rl = [-car{i}.l_r*cos(psi_car{i}(cnt)) - car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), -car{i}.l_r*sin(psi_car{i}(cnt)) + car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
            p_fr = [car{i}.l_f*cos(psi_car{i}(cnt))  + car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), car{i}.l_f*sin(psi_car{i}(cnt)) - car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
            p_rr = [-car{i}.l_r*cos(psi_car{i}(cnt)) + car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), -car{i}.l_r*sin(psi_car{i}(cnt)) - car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
            
            p_aerofl{i} = [(car{i}.l_f + car{i}.wingLength_f)*cos(psi_car{i}(cnt))  - car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), (car{i}.l_f + car{i}.wingLength_f)*sin(psi_car{i}(cnt)) + car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
            p_aerofr{i} = [(car{i}.l_f + car{i}.wingLength_f)*cos(psi_car{i}(cnt))  + car{i}.track/2*sin(psi_car{i}(cnt)) + x_car{i}(cnt), (car{i}.l_f + car{i}.wingLength_f)*sin(psi_car{i}(cnt)) - car{i}.track/2*cos(psi_car{i}(cnt)) + y_car{i}(cnt)];
            
            h_car_COM{i}.XData = x_car{i}(cnt);
            h_car_COM{i}.YData = y_car{i}(cnt);
            h_car_body{i}.XData = [p_f(1) p_r(1)];
            h_car_body{i}.YData = [p_f(2) p_r(2)];

            h_car_wing_f{i}.XData = [p_aerofl{i}(1) p_aerofr{i}(1)];
            h_car_wing_f{i}.YData = [p_aerofl{i}(2) p_aerofr{i}(2)];
            if p.Results.ColoredTrackLine
                h_trackLine{i}.XData = [x_car{i}(1:cnt)'; x_car{i}(1:cnt)'];
                h_trackLine{i}.YData = [y_car{i}(1:cnt)'; y_car{i}(1:cnt)'];
                h_trackLine{i}.ZData = zeros(size(h_trackLine{i}.YData));
                h_trackLine{i}.CData = [v_car{i}(1:cnt)';v_car{i}(1:cnt)']*3.6;
            else
                h_trackLine{i}.XData = x_car{i}(1:cnt);
                h_trackLine{i}.YData = y_car{i}(1:cnt);
            end

            % Subplot 2

            if car{i}.tracks == 2
                h_car_COM2{i}.XData = x_car{i}(cnt);
                h_car_COM2{i}.YData = y_car{i}(cnt);
                h_car_body2{i}.XData = [p_f(1) p_r(1)];
                h_car_body2{i}.YData = [p_f(2) p_r(2)];

                h_car_Wheel_fl{i}.XData = 1*car{i}.wheelradius*[cos(psi_car{i}(cnt) + delta_f{i}(cnt)) -cos(psi_car{i}(cnt) + delta_f{i}(cnt))] + p_fl(1);
                h_car_Wheel_fl{i}.YData = 1*car{i}.wheelradius*[sin(psi_car{i}(cnt) + delta_f{i}(cnt)) -sin(psi_car{i}(cnt) + delta_f{i}(cnt))] + p_fl(2);

                h_car_Wheel_fr{i}.XData = 1*car{i}.wheelradius*[cos(psi_car{i}(cnt) + delta_f{i}(cnt)) -cos(psi_car{i}(cnt) + delta_f{i}(cnt))] + p_fr(1);
                h_car_Wheel_fr{i}.YData = 1*car{i}.wheelradius*[sin(psi_car{i}(cnt) + delta_f{i}(cnt)) -sin(psi_car{i}(cnt) + delta_f{i}(cnt))] + p_fr(2);

                h_car_Wheel_rl{i}.XData = 1*car{i}.wheelradius*[cos(psi_car{i}(cnt) + delta_r{i}(cnt)) -cos(psi_car{i}(cnt) + delta_r{i}(cnt))] + p_rl(1);
                h_car_Wheel_rl{i}.YData = 1*car{i}.wheelradius*[sin(psi_car{i}(cnt) + delta_r{i}(cnt)) -sin(psi_car{i}(cnt) + delta_r{i}(cnt))] + p_rl(2);

                h_car_Wheel_rr{i}.XData = 1*car{i}.wheelradius*[cos(psi_car{i}(cnt) + delta_r{i}(cnt)) -cos(psi_car{i}(cnt) + delta_r{i}(cnt))] + p_rr(1);
                h_car_Wheel_rr{i}.YData = 1*car{i}.wheelradius*[sin(psi_car{i}(cnt) + delta_r{i}(cnt)) -sin(psi_car{i}(cnt) + delta_r{i}(cnt))] + p_rr(2);

                h_car_wing_f{i}.XData = [p_aerofl{i}(1) p_aerofr{i}(1)];
            h_car_wing_f{i}.YData = [p_aerofl{i}(2) p_aerofr{i}(2)];

            else
                h_car_COM2{i}.XData = x_car{i}(cnt);
                h_car_COM2{i}.YData = y_car{i}(cnt);
                h_car_body2{i}.XData = [p_f(1) p_r(1)];
                h_car_body2{i}.YData = [p_f(2) p_r(2)];
                h_traj2{i}.XData = x_car{i}(1:cnt);
                h_traj2{i}.YData = y_car{i}(1:cnt);

                h_car_frontwheel2{i}.XData = 1*car{i}.wheelradius*[cos(psi_car{i}(cnt) + delta_f{i}(cnt)) -cos(psi_car{i}(cnt) + delta_f{i}(cnt))] + p_f(1);
                h_car_frontwheel2{i}.YData = 1*car{i}.wheelradius*[sin(psi_car{i}(cnt) + delta_f{i}(cnt)) -sin(psi_car{i}(cnt) + delta_f{i}(cnt))] + p_f(2);

                h_car_rearwheel2{i}.XData = 1*car{i}.wheelradius*[cos(psi_car{i}(cnt) + delta_r{i}(cnt)) -cos(psi_car{i}(cnt) + delta_r{i}(cnt))] + p_r(1);
                h_car_rearwheel2{i}.YData = 1*car{i}.wheelradius*[sin(psi_car{i}(cnt) + delta_r{i}(cnt)) -sin(psi_car{i}(cnt) + delta_r{i}(cnt))] + p_r(2);
            end
            % Tire forces
            if p.Results.ShowTireForces

                if car{i}.tracks ==2
                    h_Ffl{i}.XData = p.Results.TireForceMaxSize*[0, cos(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffl_x{i}(cnt) - sin(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffl_y{i}(cnt)]/ftire_static{i} + p_fl(1);
                    h_Ffl{i}.YData = p.Results.TireForceMaxSize*[0, sin(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffl_x{i}(cnt) + cos(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffl_y{i}(cnt)]/ftire_static{i} + p_fl(2);
                    h_Ffr{i}.XData = p.Results.TireForceMaxSize*[0, cos(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffr_x{i}(cnt) - sin(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffr_y{i}(cnt)]/ftire_static{i} + p_fr(1);
                    h_Ffr{i}.YData = p.Results.TireForceMaxSize*[0, sin(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffr_x{i}(cnt) + cos(psi_car{i}(cnt) + delta_f{i}(cnt))*Ffr_y{i}(cnt)]/ftire_static{i} + p_fr(2);

                    h_Frl{i}.XData = p.Results.TireForceMaxSize*[0, cos(psi_car{i}(cnt) + delta_r{i}(cnt))*Frl_x{i}(cnt) - sin(psi_car{i}(cnt) + delta_r{i}(cnt))*Frl_y{i}(cnt)]/ftire_static{i} + p_rl(1);
                    h_Frl{i}.YData = p.Results.TireForceMaxSize*[0, sin(psi_car{i}(cnt) + delta_r{i}(cnt))*Frl_x{i}(cnt) + cos(psi_car{i}(cnt) + delta_r{i}(cnt))*Frl_y{i}(cnt)]/ftire_static{i} + p_rl(2);
                    h_Frr{i}.XData = p.Results.TireForceMaxSize*[0, cos(psi_car{i}(cnt) + delta_r{i}(cnt))*Frr_x{i}(cnt) - sin(psi_car{i}(cnt) + delta_r{i}(cnt))*Frr_y{i}(cnt)]/ftire_static{i} + p_rr(1);
                    h_Frr{i}.YData = p.Results.TireForceMaxSize*[0, sin(psi_car{i}(cnt) + delta_r{i}(cnt))*Frr_x{i}(cnt) + cos(psi_car{i}(cnt) + delta_r{i}(cnt))*Frr_y{i}(cnt)]/ftire_static{i} + p_rr(2);

                    if(ftire_max_fl{i}(cnt)> ftire_static{i})
                        h_Ffl_max_circle{i}.Color = "Green";
                    else
                        h_Ffl_max_circle{i}.Color = "Red";
                    end

                    if(ftire_max_fr{i}(cnt)> ftire_static{i})
                        h_Ffr_max_circle{i}.Color = "Green";
                    else
                        h_Ffr_max_circle{i}.Color = "Red";
                    end

                    if(ftire_max_rl{i}(cnt)> ftire_static{i})
                        h_Frl_max_circle{i}.Color = "Green";
                    else
                        h_Frl_max_circle{i}.Color = "Red";
                    end


                    if(ftire_max_rr{i}(cnt)> ftire_static{i})
                        h_Frr_max_circle{i}.Color = "Green";
                    else
                        h_Frr_max_circle{i}.Color = "Red";
                    end


                    h_Ffl_max_circle{i}.XData = (ftire_max_fl{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - (ftire_max_fl{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fl(1);
                    h_Ffl_max_circle{i}.YData = (ftire_max_fl{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + (ftire_max_fl{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fl(2);
                    h_Ffr_max_circle{i}.XData = (ftire_max_fr{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - (ftire_max_fr{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fr(1);
                    h_Ffr_max_circle{i}.YData = (ftire_max_fr{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + (ftire_max_fr{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fr(2);

                    h_Frl_max_circle{i}.XData = (ftire_max_rl{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - (ftire_max_rl{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rl(1);
                    h_Frl_max_circle{i}.YData = (ftire_max_rl{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + (ftire_max_rl{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rl(2);
                    h_Frr_max_circle{i}.XData = (ftire_max_rr{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - (ftire_max_rr{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rr(1);
                    h_Frr_max_circle{i}.YData = (ftire_max_rr{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + (ftire_max_rr{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rr(2);


                    h_Ffl_static_circle{i}.XData = maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fl(1);
                    h_Ffl_static_circle{i}.YData = maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fl(2);
                    h_Ffr_static_circle{i}.XData = maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fr(1);
                    h_Ffr_static_circle{i}.YData = maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_fr(2);
                    
                    h_Frl_static_circle{i}.XData = maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rl(1);
                    h_Frl_static_circle{i}.YData = maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rl(2);
                    h_Frr_static_circle{i}.XData = maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rr(1);
                    h_Frr_static_circle{i}.YData = maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_rr(2);
                    
                    h_car_wing_f{i}.XData = [p_aerofl{i}(1) p_aerofr{i}(1)];
                    h_car_wing_f{i}.YData = [p_aerofl{i}(2) p_aerofr{i}(2)];
                else
                    h_Ff{i}.XData = p.Results.TireForceMaxSize*[0, cos(psi_car{i}(cnt) + delta_f{i}(cnt))*Ff_x{i}(cnt) - sin(psi_car{i}(cnt) + delta_f{i}(cnt))*Ff_y{i}(cnt)]/ftire_static{i} + p_f(1);
                    h_Ff{i}.YData = p.Results.TireForceMaxSize*[0, sin(psi_car{i}(cnt) + delta_f{i}(cnt))*Ff_x{i}(cnt) + cos(psi_car{i}(cnt) + delta_f{i}(cnt))*Ff_y{i}(cnt)]/ftire_static{i} + p_f(2);
                    h_Fr{i}.XData = p.Results.TireForceMaxSize*[0, cos(psi_car{i}(cnt) + delta_r{i}(cnt))*Fr_x{i}(cnt) - sin(psi_car{i}(cnt) + delta_r{i}(cnt))*Fr_y{i}(cnt)]/ftire_static{i} + p_r(1);
                    h_Fr{i}.YData = p.Results.TireForceMaxSize*[0, sin(psi_car{i}(cnt) + delta_r{i}(cnt))*Fr_x{i}(cnt) + cos(psi_car{i}(cnt) + delta_r{i}(cnt))*Fr_y{i}(cnt)]/ftire_static{i} + p_r(2);

                    if(ftire_max_f{i}(cnt)> ftire_static{i})
                        h_Ff_max_circle{i}.Color = "Green";
                    else
                        h_Ff_max_circle{i}.Color = "Red";
                    end

                    if(ftire_max_r{i}(cnt)> ftire_static{i})
                        h_Fr_max_circle{i}.Color = "Green";
                    else
                        h_Fr_max_circle{i}.Color = "Red";
                    end


                    h_Ff_max_circle{i}.XData = (ftire_max_f{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - (ftire_max_f{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_f(1);
                    h_Ff_max_circle{i}.YData = (ftire_max_f{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + (ftire_max_f{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_f(2);
                    h_Fr_max_circle{i}.XData = (ftire_max_r{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - (ftire_max_r{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_r(1);
                    h_Fr_max_circle{i}.YData = (ftire_max_r{i}(cnt)/ftire_static{i}) * maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + (ftire_max_r{i}(cnt)/ftire_static{i}) *maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_r(2);

                    h_Ff_static_circle{i}.XData = maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_f(1);
                    h_Ff_static_circle{i}.YData = maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_f(2);
                    h_Fr_static_circle{i}.XData = maxForceCircle(1, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) - maxForceCircle(2, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_r(1);
                    h_Fr_static_circle{i}.YData = maxForceCircle(1, :) * sin(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2) + maxForceCircle(2, :) * cos(psi_car{i}(cnt) + delta_f{i}(cnt) + pi/2)  + p_r(2);
                end
            end
        end

        if N_exps == 1
            % Subplot 3
            h_Fflx_bar.YData(3:4) = 0.9*Ffl_x{1}(cnt)/ftire_max_fl{1}(cnt)/2 + F_y0;
            if u_visu{1}(cnt, 3) > 0
                h_Fflx_bar.FaceColor = [0 1 0];
            else
                h_Fflx_bar.FaceColor = [1 0 0];
            end
            h_Ffrx_bar.YData(3:4) = 0.9*Frl_x{1}(cnt)/ftire_max_rl{1}(cnt)/2 + F_y0;
            if u_visu{1}(cnt, 4) > 0
                h_Ffrx_bar.FaceColor = [0 1 0];
            else
                h_Ffrx_bar.FaceColor = [1 0 0];
            end
           % ggv = plot(Ffl_x{1}(cnt)+ Ffr_x{1}(cnt)+Frl_x{1}(cnt)+Frr_x{1}(cnt))
            h_speedometer.String = sprintf('%3d', round(sqrt(vx_car{1}(cnt)^2 + vy_car{1}(cnt)^2)*3.3));
        end

        h_sp2.XLim = x_car{1}(cnt) + p.Results.MagnifiedWindowSize/2*[-1 1];
        h_sp2.YLim = y_car{1}(cnt) + p.Results.MagnifiedWindowSize/2*[-1 1];

        h_time.String = sprintf('%02.2f',   t_visu{1}(cnt));

        dt = toc;

        drawnow;

        if ~isempty(p.Results.FileName)
            % Capture the plot as an image
            frame = getframe(h);

            switch lower(ext)
                case '.gif'
                    im = frame2im(frame);
                    [imind,cm] = rgb2ind(im,256);

                    % Write to the GIF File
                    if cnt == 1
                        imwrite(imind, cm, p.Results.FileName, 'gif', 'Loopcount',inf, 'DelayTime',1/p.Results.FPS);
                    else
                        imwrite(imind, cm, p.Results.FileName, 'gif', 'WriteMode', 'append', 'DelayTime',1/p.Results.FPS);
                    end
                case '.avi'
                    writeVideo(vidObj, getframe(gcf));
            end
        end

        if dt < 1/p.Results.FPS
            pause(1/p.Results.FPS - dt);
        end
    end

    % Save (or close) the animation file if the chosen file format needs it
    if ~isempty(p.Results.FileName)
        switch lower(ext)
            case '.gif'
            case '.avi'
                close(vidObj);
        end
    end
end