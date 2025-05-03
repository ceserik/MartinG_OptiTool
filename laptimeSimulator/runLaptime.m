function [time,expdata,opti,sol] = runLaptime(car,trackName)
%RUNLAPTIME Summary of this function goes here
%   returns time for lap

tic

%%                   Optimization parameters
%%%%%%%%%%%%%%%%%%%% Optimization parameters %%%%%%%%%%%%%%%%%%%%
optparams.method      = 'rk4';  % 'euler', 'rk4', 'collocation' | euler - the simplest, the less accurate, 'collocation' - more accurate, rather robust, 'rk4' - the most accurate, the less stable
optparams.samplingInterval = 0.1;   % determines approximately the distance between sampled arc lengths along the trajectory. The smaller the distance the larger the optimization problem becomes
% o.25 sampling pre suspension 0.1
optparams.steeredAxle = car.steeredAxle; % 'front', 'rear', 'both'
optparams.drivenAxle  = car.drivenAxle; % 'front', 'rear', 'both'

% Constraints on maximum and minimum force on tires in x direction
optparams.Ff_max      = car.Mf_max/car.wheelradius;
optparams.Ff_min      = -2*optparams.Ff_max;
optparams.Fr_max      = car.Mr_max/car.wheelradius;
optparams.Fr_min      = -2*optparams.Fr_max;

%optparams.Ff_max      = Inf;
%optparams.Ff_min      = -Inf;
%optparams.Fr_max      = Inf;
%optparams.Fr_min      = -Inf;
if car.steeringAngle_max < Inf
    optparams.delta_scale = car.steeringAngle_max; % Scaling factor for the steering angles
else
    optparams.delta_scale = pi/2; % Scaling factor for the steering angles
end

if car.Yt_max <Inf && car.Yt_max ~=0
    optparams.Yt_scale = car.Yt_max;
else
    optparams.Yt_scale = 1;
end

optparams.gepd_scale = car.gepdMaxPower;

optparams.alpha_sat   = 5/180*pi; % Maximum allowed tire slip angle
optparams.F_scale     = car.ftire(optparams.alpha_sat,car.m/2 * car.g,car);  % Scaling factor for the forces on tires

optparams.vel_init    = 60/3.6; % Velocity used for initialization [m/s] - this parameters is used only if optparams.init_from_sim is set to false
optparams.n0          = [];     % Position on the starting line (if empty, the car can start anywhere on the line) [m]
optparams.vx0         = [];     % Velocity on the starting line (if empty, the velocity of the car is determined by the optimization solver) [m/s]

optparams.init_from_sim = true; % If true, we let a retired old guy to drive through the track (a really slugish and simple controller keeping a slowly riding car inside the track) and initialize the optimization problem with this data

% Not all optmization parameters are set and stored in 'optparams' here.
% Some parameters are set later on in this script.

%%%%%%%%%%%%%%%%%%% Track parameters %%%%%%%%%%%%%%%%
track.name = (trackName); % 'FSCZ2023' 'doubleTurn', 'doubleTurn2', 'experimental', 'experimental2', 'eight', 'sine', 'blackwood', 'simoncelli', 'accel'

%% Track data

switch track.name
    case 'doubleTurn'
        track.w_l = 3; % Width of the track [m]
        track.w_r = 3; % Width of the track [m]

        R1 = 6; % radius of the first turn [m]
        R2 = 5; % radius of the second turn [m]
        l_straight = 20; % length of the straight at the begining of the track [m]

        t1 = linspace(pi,0,100);
        t2 = linspace(pi,2*pi,100);
        x_smpl_R1 = R1*cos(t1)-R1;
        y_smpl_R1 = R1*sin(t1);

        x_smpl_R2 = R2*cos(t2(2:end))+R2;
        y_smpl_R2 = R2*sin(t2(2:end));

        y_smpl_straight1 = -l_straight:-1;
        x_smpl_straight1 = -2*R1*ones(size(y_smpl_straight1));

        x_smpl = [x_smpl_straight1, x_smpl_R1, x_smpl_R2];
        y_smpl = [y_smpl_straight1, y_smpl_R1, y_smpl_R2];

        track.closed = false;
        optparams.FSdiscipline = false;

        smooth_factor = 1e1;
    case 'doubleTurn2'
        track.w_l = 3; % Width of the track [m]
        track.w_r = 3; % Width of the track [m]

        R1 = 6; % radius of the first turn [m]
        R2 = 10; % radius of the second turn [m]
        l_straight1 = 10; % length of the straight at the begining of the track [m]
        l_straight2 = 15; % length of the straight at the begining of the track [m]

        t1 = linspace(pi,pi/2,100);
        t2 = linspace(pi/2,0,100);
        t3 = linspace(pi,2*pi,100);

        x_smpl_R1 = R1*cos(t1) - R1 - l_straight2;
        y_smpl_R1 = R1*sin(t1);

        x_smpl_straight2 = -(l_straight2+R1-1):-(R1+1);
        y_smpl_straight2 = R1*ones(size(x_smpl_straight2));

        x_smpl_R2 = R1*cos(t2)-R1;
        y_smpl_R2 = R1*sin(t2);

        x_smpl_R3 = R2*cos(t3(2:end))+R2;
        y_smpl_R3 = R2*sin(t3(2:end));

        y_smpl_straight1 = -l_straight1:-1;
        x_smpl_straight1 = -2*R1*ones(size(y_smpl_straight1)) - l_straight2;

        x_smpl = [x_smpl_straight1, x_smpl_R1, x_smpl_straight2, x_smpl_R2, x_smpl_R3];
        y_smpl = [y_smpl_straight1, y_smpl_R1, y_smpl_straight2, y_smpl_R2, y_smpl_R3];

        track.closed = false;
        optparams.FSdiscipline = false;

        smooth_factor = 1e4;
    case 'experimental'
        track.w_l = 1.5; % Width of the track [m]
        track.w_r = 1.5; % Width of the track [m]

        w_x = 50; % width of the track along x axis
        w_y = 30; % width of the track along y axis
        t = (0:0.005:1)';
        x_smpl = w_x*sin(2*pi*t);
        y_smpl = w_y*(cos(2*pi*t) + 0.5*sin(2*pi*3*t));
        optparams.FSdiscipline = false;

        track.closed = true;
        smooth_factor = 5e1;
    case 'experimental2'
        track.w_l = 5; % Width of the track [m]
        track.w_r = 5; % Width of the track [m]

        w_x = 45; % width of the track along x axis
        w_y = 25; % width of the track along y axis
        t = (0:0.001:1)';
        x_smpl = w_x*sin(2*pi*t);
        y_smpl = w_y*(0.5*cos(2*pi*3*t));

        track.closed = true;
        optparams.FSdiscipline = false;

        smooth_factor = 1e2;
    case 'eight'
        track.w_l = 2; % Width of the track [m]
        track.w_r = 2; % Width of the track [m]

        w_x = 50; % width of the track along x axis
        w_y = 30; % width of the track along y axis
        t = (0:0.01:2*pi)';
        x_smpl = w_x*cos(t);
        y_smpl = w_y*sin(2*t);

        track.closed = true;
        optparams.FSdiscipline = false;

        smooth_factor = 1e2;
    case 'sine'
        track.w_l = 5; % Width of the track [m]
        track.w_r = 5; % Width of the track [m]

        w_x = 100; % width of the track along x axis
        w_y = 40; % width of the track along y axis
        % Sine trajectory
        x_smpl = (0:.1:w_x)';
        y_smpl = w_y/2*sin(2*pi/x_smpl(end)*x_smpl);

        track.closed = false;
        optparams.FSdiscipline = false;
        smooth_factor = 1e2;

    case 'blackwood'
        track.w_l = 5; % Width of the track [m]
        track.w_r = 5; % Width of the track [m]
        track.closed = true;

        smooth_factor = 1e5;

        load('.\tracks\blackwood.mat')

        x_smpl = x_smpl(end:-1:1);
        y_smpl = y_smpl(end:-1:1);
        optparams.FSdiscipline = false;

    case 'simoncelli'
        track.w_l = 5; % Width of the track [m]
        track.w_r = 5; % Width of the track [m]
        track.closed = true;

        smooth_factor = 5e5;

        load('.\tracks\simoncelli.mat')

        optparams.FSdiscipline = false;

    case 'FSCZ2023'
        % track was obtained from race car, now from trackwalk, therefore
        % we have to constrain track width a lot
        track.w_l = 1.0; % Width of the track [m] track width = 3m, car width = 1.2m => singltrack can move +-0.9m zvladlo fscz na 0.3 collocation
        track.w_r = 1.0; % Width of the track [m]
        track.closed = true;
        load tracks/FSCZ2023.mat;
        x_smpl  = flip(trackFSCZ.Data.Proc.x);
        y_smpl  = flip(trackFSCZ.Data.Proc.y);
        smooth_factor = 1e1;
        optparams.FSdiscipline = false;

    case 'accel'
        track.w_l = 1.5; % Width of the track [m]
        track.w_r = 1.5; % Width of the track [m]
        track.closed = false;
        l_straight = 75;
        y_smpl_straight1 = -l_straight:-1;
        x_smpl_straight1 = -2*1*ones(size(y_smpl_straight1));

        x_smpl = [x_smpl_straight1];
        y_smpl = [y_smpl_straight1];

        smooth_factor = 5e5;
        optparams.FSdiscipline = true;

    case 'skidpad'
        track.w_l = 1.5; % Width of the track [m]
        track.w_r = 1.5; % Width of the track [m]
        radius = (15.25 + 1.5)/2;% radius of centerline inside skidpad
        straight = 10; % length of straight

        y_smpl_straight1 = -straight:0;
        x_smpl_straight1 = -2*radius*ones(size(y_smpl_straight1));

        t1 = linspace(pi,-3*pi,200);
        t1 = t1(2:end);
        x_smpl_R1 = radius*cos(t1)-radius;
        y_smpl_R1 = radius*sin(t1);

        y_smpl_straight2 = 10;
        x_smpl_straight2 = -2*radius*ones(size(y_smpl_straight2));

        x_smpl = [x_smpl_straight1, x_smpl_R1, -x_smpl_R1-4*radius x_smpl_straight2];
        y_smpl = [y_smpl_straight1, y_smpl_R1, y_smpl_R1 y_smpl_straight2];
        smooth_factor = 1;
        optparams.FSdiscipline = false;
        track.closed = false;

    otherwise
        error('Unknown trajectory was chosen!')
end

% Get sampled trajectory data (position, arc length, orientation,
% curviture) from the sampled points x_smpl and y_smpl
% [x_traj, y_traj, s_traj, th_traj, C_traj] = curveToPath(x_smpl, y_smpl);
disp('smootingujem trat')
[x_traj, y_traj, s_traj, th_traj, C_traj] = smoothTrackByOCP(x_smpl, y_smpl, smooth_factor, track.closed, 1);
track.s0 = s_traj(1);
track.sf = s_traj(end);

% Define a function returning trajectory data for an arbitrary arc length s
track.fcurve = @(s) deal(interp1(s_traj, x_traj,  s), ...
    interp1(s_traj, y_traj,  s), ...
    interp1(s_traj, th_traj, s), ...
    interp1(s_traj, C_traj,  s));

% Visualize the trajectory
figure(1)
clf
plotTrack(track, track.s0:track.sf, 0)
hold on
%plot(x_smpl, y_smpl, '.')
%plot(x_traj, y_traj, '.')
%legend('Sampled points', 'Resulting trajectory')

%title(trackName)
hold off
axis equal
%fontsize(10,"pixels")
xlabel('x [m]')
ylabel('y [m]')
drawnow
set(gcf, 'Position',  [0, 0, 600*0.8, 500*0.8])
%saveas(gcf,"fnders/FSCZ2023.eps",'epsc')
%% Trajectory optimization
% Fixed sampling of the arc length
s_opt = linspace(track.s0, track.sf, round(1/optparams.samplingInterval*(track.sf-track.s0)));


switch track.name 

    case 'skidpad'
        s_opt = pokus_s_opt(s_traj, C_traj, 0.1, 0.3);
    case 'accel'
        s_opt = pokus_s_opt(s_traj, C_traj, 0.1, 0.1);
    case 'endurance'
        s_opt = pokus_s_opt(s_traj, C_traj, 0.1, 0.5);
    case 'autoX'
        s_opt = pokus_s_opt(s_traj, C_traj, 0.1, 0.5);
    otherwise
        s_opt = pokus_s_opt(s_traj, C_traj, 0.1, 0.5);
end

% Variable sampling of the arc length based on the local curviture (Experimental)


[x_track, y_track, th_track, C_track] = track.fcurve(s_opt);

optparams.N           = numel(s_opt)-1;
optparams.closedTrack = track.closed; % If true, we add a constraint on equality of the state at the begining and end of the track

% Initialize the optimization problem
opti = casadi.Opti();

% define decision variables
Z = opti.variable(8, optparams.N+1); % states
z_vx   = Z(1, :);
z_vy   = Z(2, :);
z_psi  = Z(3, :);
z_dpsi = Z(4, :);
z_n    = Z(5, :);
z_t    = Z(6, :);
z_Fz   = Z(7, :);
z_Ltl  = Z(8, :);

if car.tracks == 2
    if car.PowertrainType ~= "2WDxDD"
        U = opti.variable(8, optparams.N); % controls
        U_delta_f  = U(1, :);
        U_delta_r  = U(2, :);
        U_Fmfl     = U(3, :);
        U_Fmfr     = U(4, :);
        U_Fmrl     = U(5, :);
        U_Fmrr     = U(6, :);
        U_gepd     = U(7, :);
        U_Fbrake   = U(8, :);

        U_Ffl      = U_Fmfl + U_Fbrake/2*car.BrakeBalance;
        U_Ffr      = U_Fmfr + U_Fbrake/2*car.BrakeBalance;
        U_Frl      = U_Fmrl + U_Fbrake/2*(1-car.BrakeBalance);
        U_Frr      = U_Fmrr + U_Fbrake/2*(1-car.BrakeBalance);
        opti.subject_to(U_Fbrake <= 0);

        U2 = [U_delta_f;
            U_delta_r;
            U_Ffl %+ U_Fbrake/2*car.BrakeBalance
            U_Ffr %+ U_Fbrake/2*car.BrakeBalance
            U_Frl %+ U_Fbrake/2*(1-car.BrakeBalance)
            U_Frr %+ U_Fbrake/2*(1-car.BrakeBalance)
            U_gepd
            U_Fbrake];
    else
        U = opti.variable(6, optparams.N); % controls
        U_delta_f  = U(1, :);
        U_delta_r  = U(2, :);
        U_Fmf      = U(3, :);
        U_Fmr      = U(4, :);
        U_gepd     = U(5, :);
        U_Fbrake   = U(6, :);

        U_Ffl      = U_Fbrake/4;
        U_Ffr      = U_Fbrake/4;
        U_Frl      = U_Fmr + U_Fbrake/4;
        U_Frr      = U_Fmr + U_Fbrake/4;

        U_Fmfl     = U_Fmf;
        U_Fmfr     = U_Fmf;
        U_Fmrl     = U_Fmr;
        U_Fmrr     = U_Fmr;

        U = [U_delta_f
            U_delta_r
            U_Ffl
            U_Ffr
            U_Frl
            U_Frr
            U_gepd
            U_Fbrake];
    end
else
    U = opti.variable(5, optparams.N); % controls
    U_delta_f = U(1, :);
    U_delta_r = U(2, :);
    U_Ff      = U(3, :);
    U_Fr      = U(4, :);
    U_Yt      = U(5, :);
end
% Initialization of the decision variables
if optparams.init_from_sim
    % Use a simple controller keeping a slowly riding car inside the track.
    % Simulate a ride of the car with this controller and thus get a
    % feasible trajectory for the optimization problem.

    % Parameters of the controller
    vref = 5;
    P_steering = 10;
    P_vel = 50;

    % Simulate the ride with the controller
    disp("simulating controller drive")
    if car.tracks == 2
        [s_path, z_path] = ode45(@(s, z) twinRaceCar_path_ODE(s, z, [P_steering*(-z(5));0;0;0; P_vel*(vref - z(1));P_vel*(vref - z(1));0;0], car, track), s_opt, [vref; 0; th_traj(1); 0; 0; 0; 0; 0]);
    else
        [s_path, z_path] = ode45(@(s, z) raceCar_path_ODE(s, z, [P_steering*(-z(5));0;0; P_vel*(vref - z(1));0], car, track), s_opt, [vref; 0; th_traj(1); 0; 0; 0;0]);
    end


    %% Store initialization values of the states
    optparams.vx_init   = z_path(:,1)';
    optparams.vy_init   = z_path(:,2)';
    optparams.psi_init  = z_path(:,3)';
    optparams.dpsi_init = z_path(:,4)';
    optparams.n_init    = z_path(:,5)';
    optparams.t_init    = z_path(:,6)';
    optparams.Fz_init   = z_path(:,7)';
    optparams.Ltl_init  = z_path(:,8)';

    %% Store initialization values of the controls
    if car.tracks == 2
        optparams.deltaf_init = -P_steering*optparams.n_init(1:end-1)/optparams.delta_scale;
        optparams.deltar_init = zeros(1, optparams.N);
        optparams.Ffl_init    = zeros(1, optparams.N);
        optparams.Ffr_init    = zeros(1, optparams.N);
        optparams.Frl_init    = P_vel*(vref - optparams.vx_init(1:end-1))/optparams.F_scale;
        optparams.Frr_init    = P_vel*(vref - optparams.vx_init(1:end-1))/optparams.F_scale;
        optparams.gepd_init   = zeros(1, optparams.N);
        optparams.Fbrake_init = zeros(1, optparams.N);

    else
        optparams.deltaf_init = -P_steering*optparams.n_init(1:end-1)/optparams.delta_scale;
        optparams.deltar_init = zeros(1, optparams.N);
        optparams.Ff_init     = zeros(1, optparams.N);
        
        optparams.Fr_init     = P_vel*(vref - optparams.vx_init(1:end-1))/optparams.F_scale;
        optparams.Yt_init     = zeros(1, optparams.N)/optparams.Yt_scale;

    end
    %%
    %%% Show the position of the simulated ride on the track
    % car position
    x_car_opt   = -z_path(:,5).*sin(th_track') + x_track';
    y_car_opt   =  z_path(:,5).*cos(th_track') + y_track';

    % Simulate the ride once again but this use the controls obtained in
    % the previous simulation--which was carried out with the model
    % reparametrized to path as the independent variable--and use them as
    % controls in a feedforward manner for the original model (with as the
    % indepent variable). This is done just to find out how feasible is the
    % trajectory to be used for initialization.
    disp('simulating controller drive')
    ut = optparams.t_init(1:end-1)';
    if car.tracks == 2
        [~, z_time] = ode45(@(t, z) twinRaceCar_time_ODE(z, [optparams.deltaf_init'*optparams.delta_scale, optparams.deltar_init'*optparams.delta_scale, optparams.Ffl_init'*optparams.F_scale,optparams.Ffr_init'*optparams.F_scale, optparams.Frl_init'*optparams.F_scale,optparams.Frr_init'*optparams.F_scale, optparams.gepd_init'*optparams.gepd_scale,optparams.Fbrake_init' ...
            ], car, t, ut), z_path(:,6), [z_path(1,1:4)'; x_track(1); y_track(1);z_path(1,7)';z_path(1,8)']);
    else
        [~, z_time] = ode45(@(t, z) raceCar_time_ODE(z, [optparams.deltaf_init'*optparams.delta_scale, optparams.deltar_init'*optparams.delta_scale, optparams.Ff_init'*optparams.F_scale, optparams.Fr_init'*optparams.F_scale, optparams.Yt_init'], car, t, ut), z_path(:,6), [z_path(1,1:4)'; x_track(1); y_track(1);z_path(1,7)']);
    end
    x_car_time = z_time(:,5);
    y_car_time = z_time(:,6);

    figure(10)
    clf
    plotTrack(track, s_path)

    hold on
    plot(x_car_opt, y_car_opt)
    plot(x_car_time, y_car_time,'LineStyle','--')
    legend('Simulated in path', 'Simulate in time')
    title('Trajectory initialization')
    hold off
    axis equal
    drawnow
else
    % Initialize the trajectory by a guess based on the shape of the track

    % Scale down the initial velocity proportionaly to the curviture ...
    %     vel_init = optparams.vel_init*max(0.2, 1-8*abs(C_track));
    % ... or not
    vel_init = optparams.vel_init*ones(size(C_track));

    % Compute the time vector based on the velocity vector
    dt_init = diff(s_opt)./vel_init(1:end-1);
    t_init = [0; cumsum(dt_init')];
    % Compute the angular velocty of the car based on the differentiation of the orientation of the track
    dpsi_init = diff(th_track)./dt_init; dpsi_init(end+1) = dpsi_init(end);

    % Store initialization values of the states
    optparams.vx_init =     vel_init';
    optparams.vy_init =     zeros(optparams.N+1, 1);
    optparams.psi_init =    th_track';
    optparams.dpsi_init =   dpsi_init';
    optparams.n_init =      (track.w_l - track.w_r)*ones(optparams.N+1, 1);
    optparams.t_init =      t_init';
    % Store initialization values of the controls
    optparams.deltaf_init = zeros(optparams.N, 1);
    optparams.deltar_init = zeros(optparams.N, 1);
    optparams.Ff_init =     zeros(optparams.N, 1);
    optparams.Fr_init =     zeros(optparams.N, 1);
    %
end



% Objective function (minimize the final time)
opti.minimize(z_t(end));

% Dynamic constraints
if car.tracks == 2
    f = @(s, z, F) twinRaceCar_path_ODE(s, z, [optparams.delta_scale*F(1); optparams.delta_scale*F(2); optparams.F_scale*F(3); optparams.F_scale*F(4);optparams.F_scale*F(5);optparams.F_scale*F(6); optparams.gepd_scale*F(7); F(8)  ], car, track);
else
    f = @(s, z, F) raceCar_path_ODE(s, z, [optparams.delta_scale*F(1); optparams.delta_scale*F(2); optparams.F_scale*F(3); optparams.F_scale*F(4);optparams.Yt_scale*F(5) ], car, track);
end
disp("Creating dynamic constrains 0 Percent")
switch lower(optparams.method)
    case 'euler'
        % ---- Forward Euler --------
        for k=1:optparams.N % loop over control intervals
            x_next = Z(:,k) + (s_opt(k+1)-s_opt(k))*f(s_opt(k), Z(:,k), U2(:,k));
            opti.subject_to(Z(:,k+1)==x_next); % close the gaps

            if mod(k,33) == 0
                percent = k/optparams.N*100;
                disp(['Creating dynamic constraints ',num2str(percent),' Percent'])
            end
        end
    case 'rk4'
        % ---- RK4 --------
        for k=1:optparams.N % loop over control intervals
            dsk = (s_opt(k+1)-s_opt(k));
            % Runge-Kutta 4 integration
            k1 = f(s_opt(k),         Z(:,k),         U2(:,k));
            k2 = f(s_opt(k) + dsk/2, Z(:,k)+dsk/2*k1, U2(:,k));
            k3 = f(s_opt(k) + dsk/2, Z(:,k)+dsk/2*k2, U2(:,k));
            k4 = f(s_opt(k) + dsk,   Z(:,k)+dsk*k3,   U2(:,k));
            x_next = Z(:,k) + dsk/6*(k1+2*k2+2*k3+k4);
            opti.subject_to(Z(:,k+1)==x_next); % close the gaps

            if mod(k,100) == 0
                percent = k/optparams.N*100;
                disp(['Creating dynamic constraints ',num2str(percent),'    Percent'])
            end

        end
    case 'collocation'
        % ---- Collocation --------
        for k=1:optparams.N % loop over control intervals
            u1 = U2(:,k);
            if k ~=  optparams.N
                u2 = U2(:,k+1);
            else
                u2 = U2(:,k);
            end

            z1 = Z(:,k);
            z2 = Z(:,k+1);

            dsk = (s_opt(k+1) - s_opt(k));
            f1 = f(s_opt(k), z1, u1);
            f2 = f(s_opt(k+1), z2, u2);
            zc = (z1 + z2)/2 + dsk*(f1 - f2)/8;
            fc = f(s_opt(k)/2 + s_opt(k+1)/2, zc, (u1 + u2)/2);

            opti.subject_to(fc + 3*(z1-z2)/2/dsk + (f1+f2)/4 == 0); % close the gaps
            if mod(k,100) == 0
                percent = k/optparams.N*100;
                disp(['Creating dynamic constraints ',num2str(percent),'    Percent'])
            end
        end
    otherwise
        error('Unknown method was chosen!');
end

% Set which axles are steered and limit the steering angle
switch lower(optparams.steeredAxle)
    case 'front'
        % Limit the steering angle
        if car.steeringAngle_max < Inf
            opti.subject_to(-car.steeringAngle_max/optparams.delta_scale <= U_delta_f <= car.steeringAngle_max/optparams.delta_scale);
        end
        % Limit the maximum rate of change of the steering angle
        if car.steeringAngle_maxRate < Inf
            opti.subject_to( -car.steeringAngle_maxRate < diff(U_delta_f)./diff(z_t(1:end-1)) < car.steeringAngle_maxRate)
        end
        % Constrain the steering angle of the rear axle to zero
        opti.subject_to(U_delta_r==0);
    case 'rear'
        % Limit the steering angle
        if car.steeringAngle_max < Inf
            opti.subject_to(-car.steeringAngle_max/optparams.delta_scale <= U_delta_f <= car.steeringAngle_max/optparams.delta_scale);
            opti.subject_to(-car.steeringAngle_max/optparams.delta_scale <= U_delta_r <= car.steeringAngle_max/optparams.delta_scale);
        end
        % Limit the maximum rate of change of the steering angle
        if car.steeringAngle_maxRate < Inf
            opti.subject_to( -car.steeringAngle_maxRate < diff(U_delta_r)./diff(z_t(1:end-1)) < car.steeringAngle_maxRate)
        end
        % Constrain the steering angle of the rear axle to zero
        opti.subject_to(U_delta_f==0);
    case 'both'
        % Limit the steering angles
        if car.steeringAngle_max < Inf
            opti.subject_to(-car.steeringAngle_max/optparams.delta_scale <= U_delta_f <= car.steeringAngle_max/optparams.delta_scale);
            opti.subject_to(-car.steeringAngleRear_max/optparams.delta_scale <= U_delta_r <= car.steeringAngleRear_max/optparams.delta_scale);
        end
        % Limit the maximum rate of change of the steering angle
        if car.steeringAngle_maxRate < Inf
            opti.subject_to( -car.steeringAngle_maxRate < diff(U_delta_f)./diff(z_t(1:end-1)) < car.steeringAngle_maxRate)
            opti.subject_to( -car.steeringAngle_maxRate < diff(U_delta_r)./diff(z_t(1:end-1)) < car.steeringAngle_maxRate)
        end
    otherwise
        error('Unsupported value: opt_prms.steeredAxle');
end

% Constrain the generated force on tires (both braking and accelerating force)

if car.tracks == 2
    opti.subject_to(U_Fmfl <= optparams.Ff_max/optparams.F_scale);
    opti.subject_to(U_Fmfr <= optparams.Ff_max/optparams.F_scale);
    opti.subject_to(U_Fmrl <= optparams.Ff_max/optparams.F_scale);
    opti.subject_to(U_Fmrr <= optparams.Ff_max/optparams.F_scale);

    opti.subject_to(U_Fmfl >= -optparams.Ff_max/optparams.F_scale);
    opti.subject_to(U_Fmfr >= -optparams.Ff_max/optparams.F_scale);
    opti.subject_to(U_Fmrl >= -optparams.Ff_max/optparams.F_scale);
    opti.subject_to(U_Fmrr >= -optparams.Ff_max/optparams.F_scale);


    %% Choose powertrain type
    switch car.PowertrainType
        case "2WD"
            opti.subject_to(U_Fmfl == 0);
            opti.subject_to(U_Fmfr == 0);
        case "4WD"
            opti.subject_to(U_Fmfl == U_Fmfr);
            opti.subject_to(U_Fmrl == U_Fmrr);
        case "4WDTV"

        otherwise
            displ("wrong powertrain type")
    end

    %% limit change rate of change of force

    opti.subject_to( -car.Mf_slewRate/optparams.F_scale < diff(U_Ffl)./diff(z_t(1:end-1)) < car.Mf_slewRate/optparams.F_scale);
    opti.subject_to( -car.Mf_slewRate/optparams.F_scale < diff(U_Ffr)./diff(z_t(1:end-1)) < car.Mf_slewRate/optparams.F_scale);
    opti.subject_to( -car.Mf_slewRate/optparams.F_scale < diff(U_Frl)./diff(z_t(1:end-1)) < car.Mf_slewRate/optparams.F_scale);
    opti.subject_to( -car.Mf_slewRate/optparams.F_scale < diff(U_Frr)./diff(z_t(1:end-1)) < car.Mf_slewRate/optparams.F_scale);

    %opti.subject_to( -20*car.Mf_slewRate/optparams.F_scale < diff(U_Fbrake)./diff(z_t(1:end-1)) < 20*car.Mf_slewRate/optparams.F_scale);

    opti.subject_to( -car.Mf_jerk/optparams.F_scale < diff(diff(U_Ffl))./diff(z_t(1:end-2)) < car.Mf_jerk/optparams.F_scale)
    opti.subject_to( -car.Mf_jerk/optparams.F_scale < diff(diff(U_Ffr))./diff(z_t(1:end-2)) < car.Mf_jerk/optparams.F_scale)
    opti.subject_to( -car.Mf_jerk/optparams.F_scale < diff(diff(U_Frl))./diff(z_t(1:end-2)) < car.Mf_jerk/optparams.F_scale)
    opti.subject_to( -car.Mf_jerk/optparams.F_scale < diff(diff(U_Frr))./diff(z_t(1:end-2)) < car.Mf_jerk/optparams.F_scale)



else
    switch lower(optparams.drivenAxle)
        case 'front'
            % Limit the generated force
            if optparams.Ff_max < Inf
                opti.subject_to(U_Ff <= optparams.Ff_max/optparams.F_scale);
            end
            if optparams.Ff_min > -Inf
                opti.subject_to(optparams.Ff_min/optparams.F_scale <= U_Ff);
            end

            % Constrain the generated force on the rear axle to zero
            opti.subject_to(U_Fr==0);
        case 'rear'
            % Limit the generated force
            if optparams.Fr_max < Inf
                opti.subject_to(U_Fr <= optparams.Fr_max/optparams.F_scale);
            end
            if optparams.Fr_min > -Inf
                opti.subject_to(optparams.Fr_min/optparams.F_scale <= U_Fr);
            end

            % Constrain the generated force on the front axle to zero
            opti.subject_to(U_Ff==0);
        case 'both'
            % Limit the generated force on the front axle
            if optparams.Ff_max < Inf
                opti.subject_to(U_Ff <= optparams.Ff_max/optparams.F_scale);
            end
            if optparams.Ff_min > -Inf
                opti.subject_to(optparams.Ff_min/optparams.F_scale <= U_Ff);
            end

            % Limit the generated force on the rear axle
            if optparams.Fr_max < Inf
                opti.subject_to(U_Fr <= optparams.Fr_max/optparams.F_scale);
            end
            if optparams.Fr_min > -Inf
                opti.subject_to(optparams.Fr_min/optparams.F_scale <= U_Fr);
            end
        otherwise
            error('Unsupported value: opt_prms.drivenAxle');
    end

end
% Limit the slip angle
% alpha_f = optparams.delta_scale*U_delta_f - atan((z_vy(1:end-1) + car.l_f.*z_dpsi(1:end-1)) ./ z_vx(1:end-1));
% alpha_r = optparams.delta_scale*U_delta_r - atan((z_vy(1:end-1) - car.l_r.*z_dpsi(1:end-1)) ./ z_vx(1:end-1));


if car.tracks == 2
    v = [z_vx(1:end-1) z_vy(1:end-1) 0];
    dpsiv = [ 0 0 z_dpsi(1:end-1)];
    pos_fr = [  car.l_f  -car.track/2 0];
    pos_fl = [  car.l_f   car.track/2 0];
    pos_rr = [ -car.l_r -car.track/2 0];
    pos_rl = [ -car.l_r  car.track/2 0];

    %% Calculate vectors at uprights

    velocityUpright_flx = z_vx(1:end-1) -  z_dpsi(1:end-1) .* pos_fl(2);
    velocityUpright_frx = z_vx(1:end-1) -  z_dpsi(1:end-1) .* pos_fr(2);
    velocityUpright_rlx = z_vx(1:end-1) -  z_dpsi(1:end-1) .* pos_rl(2);
    velocityUpright_rrx = z_vx(1:end-1) -  z_dpsi(1:end-1) .* pos_rr(2);

    velocityUpright_fly = z_vy(1:end-1) +  z_dpsi(1:end-1) .* pos_fl(1);
    velocityUpright_fry = z_vy(1:end-1) +  z_dpsi(1:end-1) .* pos_fr(1);
    velocityUpright_rly = z_vy(1:end-1) +  z_dpsi(1:end-1) .* pos_rl(1);
    velocityUpright_rry = z_vy(1:end-1) +  z_dpsi(1:end-1) .* pos_rr(1);

    %% Calculate wheel velocity in wheel frames
    velocityWheel_flx = velocityUpright_flx.*cos(optparams.delta_scale.*U_delta_f) + velocityUpright_fly .* sin(optparams.delta_scale.*U_delta_f);
    velocityWheel_fly =-velocityUpright_flx.*sin(optparams.delta_scale.*U_delta_f) + velocityUpright_fly .* cos(optparams.delta_scale.*U_delta_f);

    velocityWheel_frx = velocityUpright_frx.*cos(optparams.delta_scale.*U_delta_f) + velocityUpright_fry .* sin(optparams.delta_scale.*U_delta_f);
    velocityWheel_fry =-velocityUpright_frx.*sin(optparams.delta_scale.*U_delta_f) + velocityUpright_fry .* cos(optparams.delta_scale.*U_delta_f);

    velocityWheel_rlx = velocityUpright_rlx.*cos(optparams.delta_scale.*U_delta_r) + velocityUpright_rly .* sin(optparams.delta_scale.*U_delta_r);
    velocityWheel_rly =-velocityUpright_rlx.*sin(optparams.delta_scale.*U_delta_r) + velocityUpright_rly .* cos(optparams.delta_scale.*U_delta_r);

    velocityWheel_rrx = velocityUpright_rrx.*cos(optparams.delta_scale.*U_delta_r) + velocityUpright_rry .* sin(optparams.delta_scale.*U_delta_r);
    velocityWheel_rry =-velocityUpright_rrx.*sin(optparams.delta_scale.*U_delta_r) + velocityUpright_rry .* cos(optparams.delta_scale.*U_delta_r);


    %% Calculate alphas
    alpha_fl = -atan2(velocityWheel_fly,velocityWheel_flx);
    alpha_fr = -atan2(velocityWheel_fry,velocityWheel_frx);
    alpha_rl = -atan2(velocityWheel_rly,velocityWheel_rlx);
    alpha_rr = -atan2(velocityWheel_rry,velocityWheel_rrx);

    %% limit alphas
    opti.subject_to(-optparams.alpha_sat < alpha_fl < optparams.alpha_sat);
    opti.subject_to(-optparams.alpha_sat < alpha_fr < optparams.alpha_sat);
    opti.subject_to(-optparams.alpha_sat < alpha_rl < optparams.alpha_sat);
    opti.subject_to(-optparams.alpha_sat < alpha_rr < optparams.alpha_sat);
    %%
    %% Torque speed map limit

    %     for i = 1:1:length(car.trqSpeedCharSpeed)-4
    %         % car.trqSpeedCharTrq
    %         % car.trqSpeedCharSpeed
    %
    %         forceLimit_fl =( car.trqSpeedCharTrq(i) - car.trqSpeedCharTrq(i+1))./...
    %             (car.trqSpeedCharSpeed(i) - car.trqSpeedCharSpeed(i+1)).*...
    %             (velocityWheel_flx./car.wheelradius - car.trqSpeedCharSpeed(i)) + car.trqSpeedCharTrq(i);
    %
    %         forceLimit_fr =( car.trqSpeedCharTrq(i) - car.trqSpeedCharTrq(i+1))./...
    %             (car.trqSpeedCharSpeed(i) - car.trqSpeedCharSpeed(i+1)).*...
    %             (velocityWheel_frx./car.wheelradius - car.trqSpeedCharSpeed(i)) + car.trqSpeedCharTrq(i);
    %
    %         forceLimit_rl =( car.trqSpeedCharTrq(i) - car.trqSpeedCharTrq(i+1))./...
    %             (car.trqSpeedCharSpeed(i) - car.trqSpeedCharSpeed(i+1)).*...
    %             (velocityWheel_rlx./car.wheelradius - car.trqSpeedCharSpeed(i)) + car.trqSpeedCharTrq(i);
    %
    %         forceLimit_rr =( car.trqSpeedCharTrq(i) - car.trqSpeedCharTrq(i+1))./...
    %             (car.trqSpeedCharSpeed(i) - car.trqSpeedCharSpeed(i+1)).*...
    %             (velocityWheel_rrx./car.wheelradius - car.trqSpeedCharSpeed(i)) + car.trqSpeedCharTrq(i);
    %
    %         opti.subject_to(U_Ffl <= forceLimit_fl/optparams.F_scale);
    %         opti.subject_to(U_Ffr <= forceLimit_fr/optparams.F_scale);
    %         opti.subject_to(U_Frl <= forceLimit_rl/optparams.F_scale);
    %         opti.subject_to(U_Frr <= forceLimit_rr/optparams.F_scale);
    %
    %         % opti.subject_to(U_Ffl <= optparams.Ff_max/optparams.F_scale);
    %
    %     end
    %x=-\left(0.38y\right)^{4}+24000
    %opti.subject_to(z_vx(1:end-1)./car.wheelradius <= (-0.38*U_Ffl*optparams.F_scale*11.46/0.205).^4 +24000*pi/30);
    %opti.subject_to(z_vx(1:end-1)./car.wheelradius <= (-0.38*U_Ffr*optparams.F_scale*11.46/0.205).^4 +24000*pi/30);
    %opti.subject_to(z_vx(1:end-1)./car.wheelradius <= (-0.38*U_Frl*optparams.F_scale*11.46/0.205).^4 +24000*pi/30);
    %opti.subject_to(z_vx(1:end-1)./car.wheelradius <= (-0.38*U_Frr*optparams.F_scale*11.46/0.205).^4 +24000*pi/30);

    %% limit  positive Power
    positivePower = ...
        velocityWheel_flx./car.wheelradius .* U_Ffl.*optparams.F_scale * car.wheelradius+...
        velocityWheel_frx./car.wheelradius .* U_Ffr.*optparams.F_scale * car.wheelradius+...
        velocityWheel_rlx./car.wheelradius .* U_Frl.*optparams.F_scale * car.wheelradius+...
        velocityWheel_rrx./car.wheelradius .* U_Frr.*optparams.F_scale * car.wheelradius;

    % opti.subject_to( positivePower <= car.PwrMax);

    %% limit negative Power
    diffabs = @(x) x.*tanh(x.*0.01);
    optparams.PowerBrake_scale = 1;
    car.losses = 0.1;

    Power_fl = velocityWheel_flx.*U_Fmfl.*optparams.F_scale;
    Power_fr = velocityWheel_frx.*U_Fmfr.*optparams.F_scale;
    Power_rl = velocityWheel_rlx.*U_Fmrl.*optparams.F_scale;
    Power_rr = velocityWheel_rrx.*U_Fmrr.*optparams.F_scale;

    %Power_total  = (Power_fl + Power_fr + Power_rl + Power_rr);
    %PowerBrake   = 1/2 *(Power_total - car.PwrMin*1/(1-car.losses) - diffabs(Power_total - car.PwrMin*1/(1-car.losses)));

    %PowerBrake_fl =  PowerBrake /2*(1 - car.BrakeBalance);
    %PowerBrake_fr =  PowerBrake /2*(1 - car.BrakeBalance);
    %PowerBrake_rl =  PowerBrake /2*(car.BrakeBalance);
    %PowerBrake_rr =  PowerBrake /2*(car.BrakeBalance);


    Loss_fl = diffabs(Power_fl) * car.losses;
    Loss_fr = diffabs(Power_fr) * car.losses;
    Loss_rl = diffabs(Power_rl) * car.losses;
    Loss_rr = diffabs(Power_rr) * car.losses;

    PowerAcp_fl = Power_fl + Loss_fl;
    PowerAcp_fr = Power_fr + Loss_fr;
    PowerAcp_rl = Power_rl + Loss_rl;
    PowerAcp_rr = Power_rr + Loss_rr;

    PowerAcp = PowerAcp_fl + PowerAcp_fr + PowerAcp_rl + PowerAcp_rr;
    opti.subject_to( car.PwrMin <= PowerAcp <= car.PwrMax)


    %PowerAcp     = Power_total - PowerBrake.* optparams.PowerBrake_scale;
    %PowerAcp     = PowerAcp + PowerAcp.*tanh(PowerAcp)*0.1;

    EnergyForLap = car.AcpCapacity*60*60*1000/22*0.8;

    %opti.subject_to( PowerBrake .* optparams.PowerBrake_scale <= 0)
    EnergyUsed = PowerAcp * diff(z_t)';

    %opti.subject_to(U_Ffl == U_Ffr)
    %opti.subject_to(U_Frl == U_Frr)





    %% Traction ellipse
    %gepd force calculated from Power and vehicle speed
    gepdForce = optparams.gepd_scale*U_gepd/car.gepdMaxPower.*(car.gepdForceCoeff(1) + z_vx(1:end-1).*car.gepdForceCoeff(2)) ;

    Fz_aero = 0.5*car.airDensity*car.CL*car.A*z_vx(1:end-1).^2 - gepdForce * car.gepdToggle;
    Fz_f = car.m/2*(1-car.COGr)  * car.g - z_Fz(1:end-1)/2 - Fz_aero/2 * (1-car.COP);
    Fz_r = car.m/2*(car.COGr)    * car.g + z_Fz(1:end-1)/2 - Fz_aero/2 * (car.COP);
    Fz_fl = car.m/2*(1-car.COGr) * car.g - z_Fz(1:end-1)/2 - Fz_aero/2 * (1-car.COP) - z_Ltl(1:end-1)/2;
    Fz_fr = car.m/2*(1-car.COGr) * car.g - z_Fz(1:end-1)/2 - Fz_aero/2 * (1-car.COP) + z_Ltl(1:end-1)/2;
    Fz_rl = car.m/2*(car.COGr)   * car.g + z_Fz(1:end-1)/2 - Fz_aero/2 * car.COP - z_Ltl(1:end-1)/2;
    Fz_rr = car.m/2*(car.COGr)   * car.g + z_Fz(1:end-1)/2 - Fz_aero/2 * car.COP + z_Ltl(1:end-1)/2;


    opti.subject_to( (U_Ffl*car.tire_fyFactor/car.tire_fxFactor./(Fz_fl * car.tire_fyFactor/optparams.F_scale)).^2 + (car.ftire(alpha_fl,Fz_fl,car)/optparams.F_scale./(Fz_fl * car.tire_fyFactor/optparams.F_scale)).^2 <= 1.^2)
    opti.subject_to( (U_Ffr*car.tire_fyFactor/car.tire_fxFactor./(Fz_fr * car.tire_fyFactor/optparams.F_scale)).^2 + (car.ftire(alpha_fr,Fz_fr,car)/optparams.F_scale./(Fz_fr * car.tire_fyFactor/optparams.F_scale)).^2 <= 1.^2)
    opti.subject_to( (U_Frl*car.tire_fyFactor/car.tire_fxFactor./(Fz_rl * car.tire_fyFactor/optparams.F_scale)).^2 + (car.ftire(alpha_rl,Fz_rl,car)/optparams.F_scale./(Fz_rl * car.tire_fyFactor/optparams.F_scale)).^2 <= 1.^2)
    opti.subject_to( (U_Frr*car.tire_fyFactor/car.tire_fxFactor./(Fz_rr * car.tire_fyFactor/optparams.F_scale)).^2 + (car.ftire(alpha_rr,Fz_rr,car)/optparams.F_scale./(Fz_rr * car.tire_fyFactor/optparams.F_scale)).^2 <= 1.^2)
    %% Car hitbox

    gamma =-z_psi + th_track;

    hitbox_fly = -(pos_fl(1) + car.wingLength_f).*sin(gamma) + pos_fl(2) .* cos(gamma);
    hitbox_fry = -(pos_fr(1) + car.wingLength_f).*sin(gamma) + pos_fr(2) .* cos(gamma);
    hitbox_rly = -pos_rl(1).*sin(gamma) + pos_rl(2) .* cos(gamma);
    hitbox_rry = -pos_rr(1).*sin(gamma) + pos_rr(2) .* cos(gamma);

    opti.subject_to(hitbox_fly + z_n <= track.w_l)
    opti.subject_to(-track.w_r <= hitbox_fry + z_n )
    opti.subject_to(hitbox_rly + z_n <= track.w_l)
    opti.subject_to(-track.w_r <= hitbox_rry + z_n )



else
    alpha_f = -atan2((z_vy(1:end-1) + car.l_f.*z_dpsi(1:end-1)).*cos(optparams.delta_scale*U_delta_f) - z_vx(1:end-1).*sin(optparams.delta_scale*U_delta_f) ,...
        (z_vy(1:end-1) + car.l_f.*z_dpsi(1:end-1)).*sin(optparams.delta_scale*U_delta_f) + z_vx(1:end-1).*cos(optparams.delta_scale*U_delta_f));
    alpha_r = -atan2((z_vy(1:end-1) - car.l_r.*z_dpsi(1:end-1)).*cos(optparams.delta_scale*U_delta_r) - z_vx(1:end-1).*sin(optparams.delta_scale*U_delta_r) ,...
        (z_vy(1:end-1) - car.l_r.*z_dpsi(1:end-1)).*sin(optparams.delta_scale*U_delta_r) + z_vx(1:end-1).*cos(optparams.delta_scale*U_delta_r));

    omega_f =  (z_vy(1:end-1) + car.l_f.*z_dpsi(1:end-1)).*sin(optparams.delta_scale*U_delta_f) + z_vx(1:end-1).*cos(optparams.delta_scale*U_delta_f) .* car.wheelradius;
    omega_r =  (z_vy(1:end-1) - car.l_r.*z_dpsi(1:end-1)).*sin(optparams.delta_scale*U_delta_r) + z_vx(1:end-1).*cos(optparams.delta_scale*U_delta_r) .* car.wheelradius;
    opti.subject_to(car.PwrMax/optparams.F_scale >= omega_f.*U_Ff + omega_r.*U_Fr)
    opti.subject_to(car.PwrMin/optparams.F_scale <= omega_f.*U_Ff + omega_r.*U_Fr)

    opti.subject_to(-optparams.alpha_sat < alpha_f < optparams.alpha_sat);
    opti.subject_to(-optparams.alpha_sat < alpha_r < optparams.alpha_sat);

    Fz_f = car.m/2 * car.g - z_Fz(1:end-1);
    Fz_r = car.m/2 * car.g + z_Fz(1:end-1);
    opti.subject_to( (U_Ff*car.tire_fyFactor/car.tire_fxFactor./(Fz_f * car.tire_fyFactor/optparams.F_scale)).^2 + (car.ftire(alpha_f,Fz_f,car)/optparams.F_scale./(Fz_f * car.tire_fyFactor/optparams.F_scale)).^2 <= 1.^2)
    opti.subject_to( (U_Fr*car.tire_fyFactor/car.tire_fxFactor./(Fz_r * car.tire_fyFactor/optparams.F_scale)).^2 + (car.ftire(alpha_r,Fz_r,car)/optparams.F_scale./(Fz_r * car.tire_fyFactor/optparams.F_scale)).^2 <= 1.^2)
    opti.subject_to(-track.w_r <= z_n <= track.w_l);
end



% Limit the maximum norm of a force on a tire, traction ellipse

% Limit the maximum perpendicular distance of the car to the track center line
% opti.subject_to(-track.width/2 <= z_n <= track.width/2);


% Limit the maximum rate of change of the steering angle
if car.steeringAngle_maxRate < Inf
    opti.subject_to( -car.steeringAngle_maxRate < diff(U_delta_f)./diff(z_t(1:end-1)) < car.steeringAngle_maxRate)
    opti.subject_to( -car.steeringAngle_maxRate < diff(U_delta_r)./diff(z_t(1:end-1)) < car.steeringAngle_maxRate)
end

% Limit maximum Yaw moment

if car.Yt_max < Inf && car.track == 1
    opti.subject_to(U_Yt > -car.Yt_max / optparams.Yt_scale)
    opti.subject_to(U_Yt < car.Yt_max / optparams.Yt_scale)
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= ((U_Yt/car.track)*optparams.Yt_scale))
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= -((U_Yt/car.track)*optparams.Yt_scale))

    %% both motors forward
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= ((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)

    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= -((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)

    % prevent one axle accelerating second braking to achieve more possible Yaw
    % torque
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= ((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale - U_Fr*optparams.F_scale)
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= -((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale - U_Fr*optparams.F_scale)

    % prevent one axle accelerating second braking to achieve more possible Yaw
    % torque
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= ((U_Yt/car.track)*optparams.Yt_scale) - U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)
    opti.subject_to( ...
        optparams.Ff_max + optparams.Fr_max >= -((U_Yt/car.track)*optparams.Yt_scale) - U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)


    %% braking
    %both wheels braking
    opti.subject_to( ...
        -optparams.Ff_min - optparams.Fr_min >= ((U_Yt/car.track)*optparams.Yt_scale) - U_Ff*optparams.F_scale - U_Fr*optparams.F_scale)
    opti.subject_to( ...
        -optparams.Ff_min - optparams.Fr_min >= -((U_Yt/car.track)*optparams.Yt_scale) - U_Ff*optparams.F_scale - U_Fr*optparams.F_scale)

    %one wheel forward
    opti.subject_to( ...
        -optparams.Ff_min - optparams.Fr_min >= ((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale - U_Fr*optparams.F_scale)
    opti.subject_to( ...
        -optparams.Ff_min - optparams.Fr_min >= -((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale - U_Fr*optparams.F_scale)

    opti.subject_to( ...
        -optparams.Ff_min - optparams.Fr_min >= ((U_Yt/car.track)*optparams.Yt_scale) - U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)
    opti.subject_to( ...
        -optparams.Ff_min - optparams.Fr_min >= -((U_Yt/car.track)*optparams.Yt_scale) - U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)


    %
    %     opti.subject_to( ...
    %         optparams.Ff_max + optparams.Fr_max >= -((U_Yt/car.track)*optparams.Yt_scale) + U_Ff*optparams.F_scale + U_Fr*optparams.F_scale)
end
%% GEPD limitations
% sum of gepd energy usage during each point/short length shouldnt be sum
% but integration I think cumtrapz

% limit gepd max power
opti.subject_to( 0<= U_gepd <= 12000./optparams.gepd_scale)
% limit gepd energy multiplication is to convert kWh to J
gepdEnergy = U_gepd * diff(z_t)';



%car.gepdSlewRate
opti.subject_to( -car.gepdSlewRate*car.gepdMaxPower./optparams.gepd_scale < diff(U_gepd)./diff(z_t(1:end-1)) < car.gepdSlewRate*car.gepdMaxPower./optparams.gepd_scale);


if car.discipline == "endurance"
    opti.subject_to(EnergyUsed /EnergyForLap <=1 );
    opti.subject_to(gepdEnergy <= car.gepdCapacity*1000*60*60 / optparams.gepd_scale/22)
end

if car.discipline == "autoX"
    optparams.closedTrack = 0;
    opti.subject_to( z_vx(1)   <= 8);
    opti.subject_to( z_psi(1)   == th_track(1));
end


if optparams.closedTrack
    % If the track is closed (it ends where it starts), we constrain the
    % final states to be equal the the initial state
    opti.subject_to( z_vx(1)   == z_vx(end));
    opti.subject_to( z_vy(1)   == z_vy(end));
    opti.subject_to( z_psi(1)   == th_track(1));
    opti.subject_to( z_psi(end)   == th_track(end));
    opti.subject_to( z_dpsi(1) == z_dpsi(end));
    opti.subject_to( z_n(1)    == z_n(end));
    opti.subject_to( z_Fz(1)    == z_Fz(end));
    opti.subject_to( z_Ltl(1)    == z_Ltl(end));
else
    % Constrain the orientation of the car at the start of the track so that the
    % car heads the same direction as the track
    %opti.subject_to( z_psi(1)   == th_track(1));
    % Constrain the orientation of the car at the end of the track so that the
    % car heads more or less the same direction as the track
    %opti.subject_to( (th_track(end)-pi/8) < z_psi(end) < (th_track(end)+pi/8));

    % Constrain the the lateral velocity of the car to zero at the begining and end of the
    % track aaah had problems with infeasibility when constrained vx(1) to zero
    %opti.subject_to( z_vy(1)    == 0);
    %opti.subject_to( z_vx(1)    == 0);
    %opti.subject_to( z_vy(end)  == 0);

    % Constrain the the initial angular velocity of the car to zero
    opti.subject_to( z_dpsi(1)  == 0);
    opti.subject_to( z_Fz(1)    == 0);
    opti.subject_to( z_Ltl(1)   == 0);
end

% Constrain the position of the car at the starting line
if ~isempty(optparams.n0)
    opti.subject_to( z_n(1)  == optparams.n0);
end

% Constrain the initial longitudinal velocity
if ~isempty(optparams.vx0)
    opti.subject_to( z_vx(1)  == optparams.vx0);
end

% Time has to start at zero and be increasing
opti.subject_to( z_t(1) > 0);
opti.subject_to( diff(z_t) > 0);
opti.subject_to( z_vx  <= car.maxSpeed);
%%% Initialize decision variables
opti.set_initial(z_vx,      optparams.vx_init);
opti.set_initial(z_vy,      optparams.vy_init);
opti.set_initial(z_psi,     optparams.psi_init);
opti.set_initial(z_dpsi,    optparams.dpsi_init);
opti.set_initial(z_n,       optparams.n_init');
opti.set_initial(z_t,       optparams.t_init);
opti.set_initial(z_Fz,      optparams.Fz_init);
opti.set_initial(z_Ltl,     optparams.Ltl_init);
%opti.set_initial(PowerBrake,      optparams.Frr_init*0);

if optparams.FSdiscipline
    %accel
    opti.subject_to( z_vx(1)   <= 8);
    opti.subject_to( z_psi(1)   == th_track(1));
    %opti.subject_to(U_Fmfl == U_Fmfr)
    %opti.subject_to(U_Fmrl == U_Fmrr)
    %opti.subject_to( z_dpsi(1)   == 0);
    opti.subject_to( z_n(1)   == 0);
    %opti.subject_to( z_dpsi(end)   == 0);
    %opti.subject_to( z_n   == 0);
end

if trackName == "skidpad"
   % opti.subject_to( z_n(end)   == 0);
   %  opti.subject_to( z_n(1)   == 0);
end


if car.tracks == 2
    opti.set_initial(U_delta_f, optparams.deltaf_init);
    opti.set_initial(U_delta_r, optparams.deltar_init);
    opti.set_initial(U_Fmfl,     optparams.Ffl_init);
    opti.set_initial(U_Fmfr,     optparams.Ffr_init);
    opti.set_initial(U_Fmrl,     optparams.Frl_init);
    opti.set_initial(U_Fmrr,     optparams.Frr_init);
    opti.set_initial(U_gepd,    0);

else
    opti.set_initial(U_delta_f, optparams.deltaf_init);
    opti.set_initial(U_delta_r, optparams.deltar_init);
    opti.set_initial(U_Ff,      optparams.Ff_init);
    opti.set_initial(U_Fr,      optparams.Fr_init);
    opti.set_initial(U_Yt,      optparams.Yt_init);
end
% Solree
p_opts = struct('expand',true);
s_opts = struct('max_iter',1000000);
opti.solver('ipopt',p_opts,s_opts); % use IPOPT solver
try
    sol = opti.solve();

    toc
    %%
    % Extract the solution
    z_opt = sol.value(Z)';
    u_opt = sol.value(U)';
catch
    z_opt = opti.debug.value(Z)';
    u_opt = opti.debug.value(U)';
end
u_opt(:,1:2) = optparams.delta_scale*u_opt(:,1:2);

if car.tracks == 2
    u_opt(:,3:6) = optparams.F_scale * u_opt(:,3:6);
    gepd      = optparams.gepd_scale * u_opt(:,7);
    u_opt(:,7) =optparams.gepd_scale * u_opt(:,7);
    u_opt(:,8) =optparams.F_scale * u_opt(:,8);
else
    u_opt(:,3:4) = optparams.F_scale*u_opt(:,3:4);
    u_opt(:,5) = optparams.Yt_scale*u_opt(:,5);
end
vx_opt    = z_opt(:,1); % longitudinal velocity of the car
vy_opt    = z_opt(:,2); % lateral velocity of the car
psi_opt   = z_opt(:,3); % orientation of the car
Dpsi_opt  = z_opt(:,4); % angular velocity of the car
n_opt     = z_opt(:,5); % perpendicular position of the car with respect to the track center line
t_opt     = z_opt(:,6); % time vector
Fz_opt    = z_opt(:,7); % Fz vector
Ft_opt    = t_opt(1:end-1); % time vector for the controls - this is just the time vector t_opt shorter by one element as we the controls are not applied at the end of the last control period (we have N values for each control input and N+1 values for each state)

% Calculate position of the car in xy-plane
x_car_opt   = -n_opt.*sin(th_track') + x_track';
y_car_opt   =  n_opt.*cos(th_track') + y_track';

% Store the data from the experiment in a structure
expdata.s = s_opt; % arc lengths
expdata.z = z_opt; % optimal states
expdata.u = u_opt; % optimal controls
expdata.carpos = [x_car_opt y_car_opt]; % xy position of the car
expdata.car = car; % car parameters
expdata.track = track; % track data
expdata.optparams = optparams; % optimization parameters

expName = sprintf('%s_S-%s_D_%s_GEPD_%d', track.name, optparams.steeredAxle, optparams.drivenAxle, car.gepdToggle);
save(fullfile('results', strcat(expName, '.mat')), 'expdata');

%%

%%% Plot the states and controls
%plotStatesControls(z_opt,t_opt,u_opt,Ft_opt)

%% Optimal trajectory check

%%% Check how the optimal trajectory of controls performs when applied to
%%% the original model (with time as the indepented variable) in a
%%% feedforward manner. If the difference between the simulated and optimal
%%% trajectory is huge, either more accurate discretization technique (e.g.
%%% RK4 instead of euler) or more discretization points should be used (larger optparams.N)

timeSimCheck(expdata)

%mydata = optdata(z_opt,u_opt,car);

[lol mydata]=twinRaceCar_path_ODE(expdata.s',expdata.z',expdata.u',car,expdata.track)
%% Animation
% Show an animation of the car going through the track
%raceCar_visu(expdata, 'ColoredTrackLine', true, 'ShowTireForce', true);


%% Record a video
try
        videoName = fullfile('results', strcat(expName, '.avi'));
    raceCar_visu(expdata, 'FileName', videoName, 'ColoredTrackLine', true, 'ShowTireForce', true);
end
%powergen = (mydata.power_fl.data +mydata.power_fr.data + mydata.power_rl.data + mydata.power_rr.data) - sol.value(PowerBrake)'*optparams.PowerBrake_scale, figure,plot(powergen),hold on , plot((mydata.power_fl.data +mydata.power_fr.data + mydata.power_rl.data + mydata.power_rr.data)),plot(sol.value(PowerBrake)'*optparams.PowerBrake_scale);

%PowerAcp = sol.value(PowerAcp);



time = t_opt(end);

end

function track = createTrack_DoubleTurn
track.w_l = 3; % Width of the track [m]
        track.w_r = 3; % Width of the track [m]

        R1 = 6; % radius of the first turn [m]
        R2 = 5; % radius of the second turn [m]
        l_straight = 20; % length of the straight at the begining of the track [m]

        t1 = linspace(pi,0,100);
        t2 = linspace(pi,2*pi,100);
        x_smpl_R1 = R1*cos(t1)-R1;
        y_smpl_R1 = R1*sin(t1);

        x_smpl_R2 = R2*cos(t2(2:end))+R2;
        y_smpl_R2 = R2*sin(t2(2:end));

        y_smpl_straight1 = -l_straight:-1;
        x_smpl_straight1 = -2*R1*ones(size(y_smpl_straight1));

        x_smpl = [x_smpl_straight1, x_smpl_R1, x_smpl_R2];
        y_smpl = [y_smpl_straight1, y_smpl_R1, y_smpl_R2];

        track.closed = false;
        optparams.FSdiscipline = false;

        smooth_factor = 1e1;
        track.name = "doubleTurn";
end
