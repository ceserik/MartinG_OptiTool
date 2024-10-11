z_opt = opti.debug.value(Z)';
u_opt = opti.debug.value(U)';


u_opt(:,1:2) = optparams.delta_scale*u_opt(:,1:2);
u_opt(:,3:4) = optparams.F_scale*u_opt(:,3:4);
u_opt(:,5)   = optparams.Yt_scale*u_opt(:,5);
gepd      = u_opt(:,7);
vx_opt    = z_opt(:,1); % longitudinal velocity of the car
vy_opt    = z_opt(:,2); % lateral velocity of the car
psi_opt   = z_opt(:,3); % orientation of the car
Dpsi_opt  = z_opt(:,4); % angular velocity of the car
n_opt     = z_opt(:,5); % perpendicular position of the car with respect to the track center line
t_opt     = z_opt(:,6); % time vector
Fz_opt    = z_opt(:,7); % Fz vector
Ltl_opt    = z_opt(:,7); % Fz vector
Ft_opt    = t_opt(1:end-1); % time vector for the controls - this is just the time vector t_opt shorter by one element as we the controls are not applied at the end of the last control period (we have N values for each control input and N+1 values for each state)


% Calculate position of the car in xy-plane
x_car_opt   = -n_opt.*sin(th_track') + x_track';
y_car_opt   =  n_opt.*cos(th_track') + y_track';

figure
plotTrack(track, track.s0:track.sf, 1)
hold on
plot(x_car_opt,y_car_opt)

xd = optdata(z_opt, u_opt, car);
xd.plotStates

expdata.s = s_opt; % arc lengths
expdata.z = z_opt; % optimal states
expdata.u = u_opt; % optimal controls
expdata.carpos = [x_car_opt y_car_opt]; % xy position of the car
expdata.car = car; % car parameters
expdata.track = track; % track data
expdata.optparams = optparams; % optimization parameters

