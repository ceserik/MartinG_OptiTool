function dz = twinRaceCar_time_ODE(z, u, car, t, t_F)
% z = [vx; vy; psi; dpsidt; X; Y], vx-longitudinal velocity, vy-lateral velocity, psi-angular position of the car with respect to the inertial frame, X and Y are coordinates of the car in the inertial frame
% u = [delta_f; delta_r; Ff; Fr], delta_f - steering angle of the front wheel, delta_r - steering angle of the rear wheel (NOT IMPLEMENTED YET), Ff - force acting on the front wheel in the longitudinal direction, Fr - force acting on the rear wheel in the longitudinal direction
% car - structure containing parameters of the car model


if nargin < 5 || isempty(t_F) || isempty(t)
    Ft = u;
else
    % Interpolate controls
    Ft = interp1(t_F, u, t, 'previous', 'extrap');
end

% Extract the controls from F
delta_f   = Ft(1);
delta_r   = Ft(2);
Fx_fl     = Ft(3);
Fx_fr     = Ft(4);
Fx_rl     = Ft(5);
Fx_rr     = Ft(6);
gepdPower = Ft(7);

%% Extract the states from z
v_x  = z(1);
v_y  = z(2);
psi  = z(3);
dpsi = z(4);
X    = z(5);
Y    = z(6);
Fz   = z(7);
Ltl  = z(8);
%% Compute the slip angles
%% initialize matrices, origin of car is in COG
v = [v_x v_y 0];
dpsiv = [ 0 0 dpsi];
pos_fr = [  car.l_f  -car.track/2 0];
pos_fl = [  car.l_f   car.track/2 0];
pos_rr = [ -car.l_r -car.track/2 0];
pos_rl = [ -car.l_r  car.track/2 0];

%% Calculate vectors at uprights
velocityUpright_fl = v + cross(dpsiv,pos_fl);
velocityUpright_fr = v + cross(dpsiv,pos_fr);
velocityUpright_rl = v + cross(dpsiv,pos_rl);
velocityUpright_rr = v + cross(dpsiv,pos_rr);

%% Define Rotation matrices
rotate_f = [
    cos(delta_f) -sin(delta_f) 0
    sin(delta_f) cos(delta_f)  0
    0            0             0
    ];
rotate_r = [
    cos(delta_r) -sin(delta_r) 0
    sin(delta_r) cos(delta_r)  0
    0            0             0
    ];

%% Calculate wheel velocity in wheel frames
velocityWheel_fl = velocityUpright_fl * rotate_f;
velocityWheel_fr = velocityUpright_fr * rotate_f;
velocityWheel_rl = velocityUpright_rl * rotate_r;
velocityWheel_rr = velocityUpright_rr * rotate_r;


%% Calculate alphas
alpha_fl = -atan2(velocityWheel_fl(2),velocityWheel_fl(1));
alpha_fr = -atan2(velocityWheel_fr(2),velocityWheel_fr(1));
alpha_rl = -atan2(velocityWheel_rl(2),velocityWheel_rl(1));
alpha_rr = -atan2(velocityWheel_rr(2),velocityWheel_rr(1));


%% Compute the forces acting on the wheels
%Fz_aero = 0.5*car.airDensity*car.CL*car.A*v_x.^2;
%gepd force implemented here
gepdForce = gepdPower/car.gepdMaxPower * (car.gepdForceCoeff(1) + v_x*car.gepdForceCoeff(2));
Fz_aero = 0.5*car.airDensity*car.CL*car.A*v_x.^2 - gepdForce*car.gepdToggle;

Fz_fl = car.m/2*(1-car.COGr) * car.g - Fz/2 - Fz_aero/2 * (1-car.COP) - Ltl/2;
Fz_fr = car.m/2*(1-car.COGr) * car.g - Fz/2 - Fz_aero/2 * (1-car.COP) + Ltl/2;
Fz_rl = car.m/2*(car.COGr)   * car.g + Fz/2 - Fz_aero/2 * car.COP - Ltl/2;
Fz_rr = car.m/2*(car.COGr)   * car.g + Fz/2 - Fz_aero/2 * car.COP + Ltl/2;

Fy_fl = car.ftire(alpha_fl,Fz_fl, car);
Fy_fr = car.ftire(alpha_fr,Fz_fr, car);
Fy_rl = car.ftire(alpha_rl,Fz_rl, car);
Fy_rr = car.ftire(alpha_rr,Fz_rr, car);

%% Projection of forces from steered wheels to uprights
projection_f = [
    cos(delta_f)      sin(delta_f)
    -sin(delta_f)     cos(delta_f)   
    ];
projection_r = [
    cos(delta_r)      sin(delta_r)
    -sin(delta_r)     cos(delta_r)   
    ];

%% Rolling resistance

Fr_fl = Fz_fl *  car.rr;                       
Fr_fr = Fz_fr *  car.rr;                         
Fr_rl = Fz_rl *  car.rr;                      
Fr_rr = Fz_rr *  car.rr;
 
 Fr = Fr_rr +Fr_rl +Fr_fl + Fr_fr;


%% Forces acting on uprights
F_fl = [Fx_fl Fy_fl] * projection_f;
F_fr = [Fx_fr Fy_fr] * projection_f;
F_rl = [Fx_rl Fy_rl] * projection_r;
F_rr = [Fx_rr Fy_rr] * projection_r;

F_drag = 0.5*car.airDensity*car.CD*car.A*v_x^2;
% Compute quantities need for the change of the position in the inertial frame
v = sqrt(v_x^2 + v_y^2);
beta = atan2(v_y, v_x);

M = (F_fr(1) - Fr_fr + F_rr(1) - Fr_rr - F_fl(1) + Fr_fl - F_rl(1) + Fr_rl)*car.track/2 +... % Forward forces create Moment
    (F_fr(2) + F_fl(2)) * car.l_f...                         % Lateral forces from front tires
   -(F_rr(2) + F_rl(2)) * car.l_r;                          % Lateral forces from rear tires

Fx = F_fr(1) + F_rr(1) + F_fl(1) + F_rl(1) - F_drag - Fr;         % Longitudinal force from wheels and drag and rolling resistance
Fy = F_fr(2) + F_rr(2) + F_fl(2) + F_rl(2);                  % Lateral force


%dvx = 1/car.m  * (F_rx*cos(delta_r) - F_ry*sin(delta_r) + F_fx*cos(delta_f) - F_fy*sin(delta_f) - 0.5*car.airDensity*car.CD*car.A*v_x^2) + dpsi*v_y;
dz = [... 
Fx/car.m  + dpsi*v_y                                                                                                                      % v_x/dt
Fy/car.m  - dpsi*v_x                      % v_y/dt  
dpsi;                                                                                                                         % psi/dt
M/car.Izz   % dpsi
v*cos(psi + beta);                                                                                                            % X/dt
v*sin(psi + beta);                                                                                                            % Y/dt
((Fx/car.m) *car.COGz/car.wheelbase*car.m-Fz)/car.Sus_t;                                                                      %Fz/dt  
((Fy/car.m) *car.COGz/car.track*car.m-Ltl)/car.Sus_t;
];

end