function carVars = twinTransformations(z,u,car)
% Extract the controls from F
delta_f   = u(:,1);
delta_r   = u(:,2);
Fx_fl     = u(:,3) + u(:,8)/2*car.BrakeBalance;
Fx_fr     = u(:,4) + u(:,8)/2*car.BrakeBalance;
Fx_rl     = u(:,5) + u(:,8)/2*(1-car.BrakeBalance);
Fx_rr     = u(:,6) + u(:,8)/2*(1-car.BrakeBalance);
gepdPower = u(:,7);
Fbrake    = u(:,8);

%% Extract the states from z

if length(z) >length(u)
    z = z(1:end-1,:);
end
vx   = z(:,1);
vy   = z(:,2);
psi  = z(:,3);
dpsi = z(:,4);
X    = z(:,5);
t    = z(:,6);
Fz   = z(:,7);
Ltl  = z(:,8);


pos_fr = [  car.l_f  -car.track/2 0];
pos_fl = [  car.l_f   car.track/2 0];
pos_rr = [ -car.l_r -car.track/2 0];
pos_rl = [ -car.l_r  car.track/2 0];

%% Calculate vectors at Carriers
velocityCarrier_flx = vx -  dpsi .* pos_fl(2);
velocityCarrier_frx = vx -  dpsi .* pos_fr(2);
velocityCarrier_rlx = vx -  dpsi .* pos_rl(2);
velocityCarrier_rrx = vx -  dpsi .* pos_rr(2);

velocityCarrier_fly = vy +  dpsi .* pos_fl(1);
velocityCarrier_fry = vy +  dpsi .* pos_fr(1);
velocityCarrier_rly = vy +  dpsi .* pos_rl(1);
velocityCarrier_rry = vy +  dpsi .* pos_rr(1);

%% Calculate wheel velocity in wheel frames
velocityWheel_flx = velocityCarrier_flx.*cos(delta_f) + velocityCarrier_fly .* sin(delta_f);
velocityWheel_fly =-velocityCarrier_flx.*sin(delta_f) + velocityCarrier_fly .* cos(delta_f);

velocityWheel_frx = velocityCarrier_frx.*cos(delta_f) + velocityCarrier_fry .* sin(delta_f);
velocityWheel_fry =-velocityCarrier_frx.*sin(delta_f) + velocityCarrier_fry .* cos(delta_f);

velocityWheel_rlx = velocityCarrier_rlx.*cos(delta_r) + velocityCarrier_rly .* sin(delta_r);
velocityWheel_rly =-velocityCarrier_rlx.*sin(delta_r) + velocityCarrier_rly .* cos(delta_r);

velocityWheel_rrx = velocityCarrier_rrx.*cos(delta_r) + velocityCarrier_rry .* sin(delta_r);
velocityWheel_rry =-velocityCarrier_rrx.*sin(delta_r) + velocityCarrier_rry .* cos(delta_r);

%% Projection of forces from steered wheels to Carriers
projection_f = [
    cos(delta_f)      sin(delta_f)
    -sin(delta_f)     cos(delta_f)
    ];
projection_r = [
    cos(delta_r)      sin(delta_r)
    -sin(delta_r)     cos(delta_r)
    ];


%% Calculate alphas
alpha_fl = -atan2(velocityWheel_fly,velocityWheel_flx);
alpha_fr = -atan2(velocityWheel_fry,velocityWheel_frx);
alpha_rl = -atan2(velocityWheel_rly,velocityWheel_rlx);
alpha_rr = -atan2(velocityWheel_rry,velocityWheel_rrx);

%% Compute the forces acting on the wheels
gepdForce = gepdPower/car.gepdMaxPower .*(car.gepdForceCoeff(1) + vx.*car.gepdForceCoeff(2));
Fz_aero = 0.5*car.airDensity*car.CL*car.A*vx.^2 - gepdForce*car.gepdToggle;

Fz_fl = car.m/2*(1-car.COGr) * car.g - Fz/2 - Fz_aero/2 * (1-car.COP) - Ltl/2;
Fz_fr = car.m/2*(1-car.COGr) * car.g - Fz/2 - Fz_aero/2 * (1-car.COP) + Ltl/2;
Fz_rl = car.m/2*(car.COGr)   * car.g + Fz/2 - Fz_aero/2 * car.COP - Ltl/2;
Fz_rr = car.m/2*(car.COGr)   * car.g + Fz/2 - Fz_aero/2 * car.COP + Ltl/2;

Fy_fl = car.ftire(alpha_fl,Fz_fl, car);
Fy_fr = car.ftire(alpha_fr,Fz_fr, car);
Fy_rl = car.ftire(alpha_rl,Fz_rl, car);
Fy_rr = car.ftire(alpha_rr,Fz_rr, car);

%% Forces acting on Carriers
F_flCx = Fx_fl.*cos(delta_f) - Fy_fl.*sin(delta_f);
F_flCy = Fx_fl.*sin(delta_f) + Fy_fl.*cos(delta_f);

F_frCx = Fx_fr.*cos(delta_f) - Fy_fr.*sin(delta_f);
F_frCy = Fx_fr.*sin(delta_f) + Fy_fr.*cos(delta_f);

F_rlCx = Fx_rl.*cos(delta_r) - Fy_rl.*sin(delta_r);
F_rlCy = Fx_rl.*sin(delta_r) + Fy_rl.*cos(delta_r);

F_rrCx = Fx_rr.*cos(delta_r) - Fy_rr.*sin(delta_r);
F_rrCy = Fx_rr.*sin(delta_r) + Fy_rr.*cos(delta_r);


F_drag = 0.5*car.airDensity*car.CD*car.A*vx.^2;
% Compute quantities need for the change of the position in the inertial frame
v = sqrt(vx.^2 + vy.^2);
beta = atan2(vy, vx);

M = (F_frCx + F_rrCx - F_flCx - F_rlCx)*car.track/2 +... % Forward forces create Moment
    (F_frCy + F_flCy) * car.l_f...                         % Lateral forces from front tires
    -(F_rrCy + F_rlCy) * car.l_r;                          % Lateral forces from rear tires

Fx = F_frCx + F_rrCx + F_flCx + F_rlCx- F_drag;         % Longitudinal force from wheels and drag
Fy = F_frCy + F_rrCy + F_flCy + F_rlCy;                  % Lateral force


ax = Fx/car.m;
ay = Fy/car.m;

%% Calc Power



carVars =zeros(length(Fx_fl),48);
%% Long forces on wheels
carVars(:,1) = Fx_fl;
carVars(:,2) = Fx_fr;
carVars(:,3) = Fx_rl;
carVars(:,4) = Fx_rr;

%% Lat forces on wheels
carVars(:,5) = Fy_fl;
carVars(:,6) = Fy_fr;
carVars(:,7) = Fy_rl;
carVars(:,8) = Fy_rr;

%% Long forces on carriers
carVars(:,9)  = F_flCx;
carVars(:,10) = F_frCx;
carVars(:,11) = F_rlCx;
carVars(:,12) = F_rrCx;

%% Lateral forces on carriers
carVars(:,13) = F_flCy;
carVars(:,14) = F_frCy;
carVars(:,15) = F_rlCy;
carVars(:,16) = F_rrCy;

%% Alphas
carVars(:,17) = alpha_fl;
carVars(:,18) = alpha_fr;
carVars(:,19) = alpha_rl;
carVars(:,20) = alpha_rr;

%% Normal forces
carVars(:,21) = Fz_fl;
carVars(:,22) = Fz_fr;
carVars(:,23) = Fz_rl;
carVars(:,24) = Fz_rr;

%% Aero forces
carVars(:,25) = F_drag;
carVars(:,26) = Fz_aero;
carVars(:,27) = 0; %Aero yaw moment
carVars(:,28) = 0; %Aero reserve

%% Velocities carriers Long
carVars(:,29) = velocityCarrier_flx;
carVars(:,30) = velocityCarrier_frx;
carVars(:,31) = velocityCarrier_rlx;
carVars(:,32) = velocityCarrier_rrx;

%% Velocities carriers Lateral
carVars(:,33) = velocityCarrier_fly;
carVars(:,34) = velocityCarrier_fry;
carVars(:,35) = velocityCarrier_rly;
carVars(:,36) = velocityCarrier_rry;

%% Velocities wheel frame Long
carVars(:,37) = velocityWheel_flx;
carVars(:,38) = velocityWheel_frx;
carVars(:,39) = velocityWheel_rlx;
carVars(:,40) = velocityWheel_rrx;

%% Velocities wheel frame Lateral
carVars(:,41) = velocityWheel_fly;
carVars(:,42) = velocityWheel_fry;
carVars(:,43) = velocityWheel_rly;
carVars(:,44) = velocityWheel_rry;

%% Combined Forces on vehicle
carVars(:,45) = Fx;
carVars(:,46) = M;
TvMomemnt = (-Fx_fl.*cos(delta_f) + Fx_fr.*cos(delta_f) - Fx_rl.*cos(delta_r) + Fx_rr.*cos(delta_r))*car.track/2;
steering = (Fy_fl + Fy_fr).*car.l_f.*cos(delta_f) - (Fy_rl + Fy_rr).*car.l_r.*cos(delta_r);
carVars(:,47) = TvMomemnt;%(F_frCx + F_rrCx - F_flCx - F_rlCx)*car.track/2; %Moment from TV
carVars(:,48) = steering;%(F_frCy + F_flCy) * car.l_f -(F_rrCy + F_rlCy) * car.l_r; %Moment from steering

%% Power on wheels
carVars(:,49) = Fx_fl .* velocityWheel_flx;
carVars(:,50) = Fx_fr .* velocityWheel_frx;
carVars(:,51) = Fx_rl .* velocityWheel_rlx;
carVars(:,52) = Fx_rr .* velocityWheel_rrx;



%% Power from acp

diffabs = @(x) x.*tanh(x.*0.01);
    Power_fl = carVars(:,49);
    Power_fr = carVars(:,50);
    Power_rl = carVars(:,51);
    Power_rr = carVars(:,52);

    Power_total  = (Power_fl + Power_fr + Power_rl + Power_rr);
    PowerBrake   = 1/2 *(Power_total - car.PwrMin*1/(1-car.losses) - diffabs(Power_total - car.PwrMin*1/(1-car.losses)));
    

    Loss_fl = diffabs(Power_fl - PowerBrake/4) * 0.1;
    Loss_fr = diffabs(Power_fr - PowerBrake/4) * 0.1;
    Loss_rl = diffabs(Power_rl - PowerBrake/4) * 0.1;
    Loss_rr = diffabs(Power_rr - PowerBrake/4) * 0.1;

    PowerAcp_fl = Power_fl + Loss_fl - PowerBrake ./4;
    PowerAcp_fr = Power_fr + Loss_fr - PowerBrake ./4;
    PowerAcp_rl = Power_rl + Loss_rl - PowerBrake ./4;
    PowerAcp_rr = Power_rr + Loss_rr - PowerBrake ./4;

    PowerAcp = PowerAcp_fl + PowerAcp_fr + PowerAcp_rl + PowerAcp_rr;


carVars(:,53) = PowerAcp ;
carVars(:,54) = Fbrake.*(velocityWheel_flx+velocityWheel_frx+velocityWheel_rlx+velocityWheel_rrx)./4;
carVars(:,55) = PowerAcp(1:end-1)' * diff(t) ;

%% losses
carVars(:,56) = (Fy_fl + Fy_fr).*sin(delta_f) + (Fy_rl + Fy_rr).*sin(delta_r);
carVars(:,57) = ax;
carVars(:,58) = ay;
carVars(:,59) = Fbrake;
end


