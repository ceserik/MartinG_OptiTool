function [dz out] = twinRaceCar_time_ODE(z, u, car, t, t_F)
% z = [vx; vy; psi; dpsidt; X; Y], vx-longitudinal velocity, vy-lateral velocity, psi-angular position of the car with respect to the inertial frame, X and Y are coordinates of the car in the inertial frame
% u = [delta_f; delta_r; Ff; Fr], delta_f - steering angle of the front wheel, delta_r - steering angle of the rear wheel (NOT IMPLEMENTED YET), Ff - force acting on the front wheel in the longitudinal direction, Fr - force acting on the rear wheel in the longitudinal direction
% car - structure containing parameters of the car model


if nargin < 5 || isempty(t_F) || isempty(t)
    Ft = u;
else
    % Interpolate controls
    Ft = interp1(t_F, u, t, 'previous', 'extrap');
end
% get length of vectors
if size(z,2) == 8
    Ft = Ft';
    z = z';
else
    Ft = Ft';
end


n = size(z,2);

%Ft = Ft';
%z = z';
if n > 1
    Ft(:,end+1) = 0;
    
    bigZero = shiftdim(zeros(n,1),-2);

    % Extract the controls from F
    MechBrake = Ft(8,:);
    delta_f   = shiftdim(Ft(1,:),-1);
    delta_r   = shiftdim(Ft(2,:),-1);
    Fx_fl     = shiftdim(Ft(3,:) + MechBrake/2*car.BrakeBalance,-1);
    Fx_fr     = shiftdim(Ft(4,:) + MechBrake/2*car.BrakeBalance,-1);
    Fx_rl     = shiftdim(Ft(5,:) + MechBrake/2*(1-car.BrakeBalance),-1);
    Fx_rr     = shiftdim(Ft(6,:) + MechBrake/2*(1-car.BrakeBalance),-1);
    gepdPower = shiftdim(Ft(7,:),-1);

    %% Extract the states from z
    v_x  = shiftdim(z(1,:),-1);
    v_y  = shiftdim(z(2,:),-1);
    psi  = shiftdim(z(3,:),-1);
    dpsi = shiftdim(z(4,:),-1);
    X    = z(5,:);
    Y    = z(6,:);
    Fz   = shiftdim(z(7,:),-1);
    Ltl  = shiftdim(z(8,:),-1);


    pos_fr = shiftdim(repmat([  car.l_f -car.track/2 0],n,1)',-1);
    pos_fl = shiftdim(repmat([  car.l_f  car.track/2 0],n,1)',-1);
    pos_rr = shiftdim(repmat([ -car.l_r -car.track/2 0],n,1)',-1);
    pos_rl = shiftdim(repmat([ -car.l_r  car.track/2 0],n,1)',-1);

else
    bigZero = 0;


    % Extract the controls from F
    MechBrake = Ft(8,:);
    delta_f   = Ft(1,:);
    delta_r   = Ft(2,:);
    Fx_fl     = Ft(3,:) + MechBrake/2*car.BrakeBalance;
    Fx_fr     = Ft(4,:) + MechBrake/2*car.BrakeBalance;
    Fx_rl     = Ft(5,:) + MechBrake/2*(1-car.BrakeBalance);
    Fx_rr     = Ft(6,:) + MechBrake/2*(1-car.BrakeBalance);
    gepdPower = Ft(7,:);

    %% Extract the states from z
    v_x  = z(1,:);
    v_y  = z(2,:);
    psi  = z(3,:);
    dpsi = z(4,:);
    X    = z(5,:);
    Y    = z(6,:);
    Fz   = z(7,:);
    Ltl  = z(8,:);

    
pos_fr = repmat([  car.l_f -car.track/2 0],n,1);
pos_fl = repmat([  car.l_f  car.track/2 0],n,1);
pos_rr = repmat([ -car.l_r -car.track/2 0],n,1);
pos_rl = repmat([ -car.l_r  car.track/2 0],n,1);

end

%% Compute the slip angles
%% initialize matrices, origin of car is in COG
v = [v_x v_y bigZero];
dpsiv = [ bigZero bigZero dpsi];


%% Calculate vectors at uprights
velocityUpright_fl = v + cross(dpsiv,pos_fl);
velocityUpright_fr = v + cross(dpsiv,pos_fr);
velocityUpright_rl = v + cross(dpsiv,pos_rl);
velocityUpright_rr = v + cross(dpsiv,pos_rr);

%% Define Rotation matrices



rotate_f = [
    cos(delta_f) -sin(delta_f) bigZero
    sin(delta_f) cos(delta_f)  bigZero
    bigZero            bigZero             bigZero
    ];
rotate_r = [
    cos(delta_r) -sin(delta_r) bigZero
    sin(delta_r) cos(delta_r)  bigZero
    bigZero      bigZero       bigZero
    ];

%% Calculate wheel velocity in wheel frames
velocityWheel_fl = mymatmul(velocityUpright_fl, rotate_f);
velocityWheel_fr = mymatmul(velocityUpright_fr, rotate_f);
velocityWheel_rl = mymatmul(velocityUpright_rl, rotate_r);
velocityWheel_rr = mymatmul(velocityUpright_rr, rotate_r);


%% Calculate alphas
alpha_fl = -atan2(seEl(velocityWheel_fl,2),seEl(velocityWheel_fl,1));
alpha_fr = -atan2(seEl(velocityWheel_fr,2),seEl(velocityWheel_fr,1));
alpha_rl = -atan2(seEl(velocityWheel_rl,2),seEl(velocityWheel_rl,1));
alpha_rr = -atan2(seEl(velocityWheel_rr,2),seEl(velocityWheel_rr,1));


%% Compute the forces acting on the wheels
%Fz_aero = 0.5*car.airDensity*car.CL*car.A*v_x.^2;
%gepd force implemented here
gepdForce = mymatmul(gepdPower/car.gepdMaxPower, (car.gepdForceCoeff(1) + v_x*car.gepdForceCoeff(2)));
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
%% Forces acting on uprights
F_fl = mymatmul([Fx_fl Fy_fl] , projection_f);
F_fr = mymatmul([Fx_fr Fy_fr] , projection_f);
F_rl = mymatmul([Fx_rl Fy_rl] , projection_r);
F_rr = mymatmul([Fx_rr Fy_rr] , projection_r);

F_drag = 0.5*car.airDensity*car.CD*car.A*v_x.^2;
% Compute quantities need for the change of the position in the inertial frame
v = sqrt(v_x.^2 + v_y.^2);
beta = atan2(v_y, v_x);

M = (seEl(F_fr,1) + seEl(F_rr,1) - seEl(F_fl,1) - seEl(F_rl,1))*car.track/2 +... % Forward forces create Moment
    (seEl(F_fr,2) + seEl(F_fl,2)) * car.l_f...                         % Lateral forces from front tires
    -(seEl(F_rr,2) + seEl(F_rl,2)) * car.l_r;                          % Lateral forces from rear tires

Fx = seEl(F_fr,1) + seEl(F_rr,1) + seEl(F_fl,1) + seEl(F_rl,1) - F_drag;         % Longitudinal force from wheels and drag
Fy = seEl(F_fr,2) + seEl(F_rr,2) + seEl(F_fl,2) + seEl(F_rl,2);                  % Lateral force


%dvx = 1/car.m  * (F_rx*cos(delta_r) - F_ry*sin(delta_r) + F_fx*cos(delta_f) - F_fy*sin(delta_f) - 0.5*car.airDensity*car.CD*car.A*v_x^2) + dpsi*v_y;
dz = [...
    Fx/car.m  + mymatmul(dpsi,v_y)                                                                                                                      % v_x/dt
    Fy/car.m  - mymatmul(dpsi,v_x)                      % v_y/dt
    dpsi;                                                                                                                         % psi/dt
    M/car.Izz   % dpsi
    mymatmul(v,cos(psi + beta))                                                                                                            % X/dt
    mymatmul(v,sin(psi + beta));                                                                                                            % Y/dt
    ((Fx/car.m) *car.COGz/car.wheelbase*car.m-Fz)/car.Sus_t;                                                                      %Fz/dt
    ((Fy/car.m) *car.COGz/car.track*car.m-Ltl)/car.Sus_t;
    ];


dz = squeeze(dz);



if n >1
    out = optdata(car);
    %% Forces acting on Carriers
    F_flCx = Fx_fl.*cos(delta_f) - Fy_fl.*sin(delta_f);
    F_flCy = Fx_fl.*sin(delta_f) + Fy_fl.*cos(delta_f);

    F_frCx = Fx_fr.*cos(delta_f) - Fy_fr.*sin(delta_f);
    F_frCy = Fx_fr.*sin(delta_f) + Fy_fr.*cos(delta_f);

    F_rlCx = Fx_rl.*cos(delta_r) - Fy_rl.*sin(delta_r);
    F_rlCy = Fx_rl.*sin(delta_r) + Fy_rl.*cos(delta_r);

    F_rrCx = Fx_rr.*cos(delta_r) - Fy_rr.*sin(delta_r);
    F_rrCy = Fx_rr.*sin(delta_r) + Fy_rr.*cos(delta_r);



    out.steering_f.data = squeeze(delta_f(1:end-1));
    out.steering_r.data = squeeze(delta_r(1:end-1));

    out.omega_fl.data = squeeze(velocityWheel_fl(:,1,1:end-1));
    out.power_fl.data = squeeze(velocityWheel_fl(:,1,1:end-1)).*u(:,3);
    out.Fy_fl.data = squeeze(Fy_fl(1:end-1));
    out.Fx_fl.data = squeeze(Fx_fl(1:end-1));
    out.Fz_fl.data = squeeze(Fz_fl(1:end-1));
    out.FxC_fl.data = squeeze(F_flCx(1:end-1));
    out.FyC_fl.data = squeeze(F_flCy(1:end-1));

    out.omega_fr.data = squeeze(velocityWheel_fr(:,1,1:end-1));
    out.power_fr.data = squeeze(velocityWheel_fr(:,1,1:end-1)).*u(:,3);
    out.Fy_fr.data = squeeze(Fy_fr(1:end-1));
    out.Fx_fr.data = squeeze(Fx_fr(1:end-1));
    out.Fz_fr.data = squeeze(Fz_fr(1:end-1));
    out.FxC_fr.data = squeeze(F_frCx(1:end-1));
    out.FyC_fr.data = squeeze(F_frCy(1:end-1));

    out.omega_rl.data = squeeze(velocityWheel_rl(:,1,1:end-1));
    out.power_rl.data = squeeze(velocityWheel_rl(:,1,1:end-1)).*u(:,3);
    out.Fy_rl.data = squeeze(Fy_rl(1:end-1));
    out.Fx_rl.data = squeeze(Fx_rl(1:end-1));
    out.Fz_rl.data = squeeze(Fz_rl(1:end-1));
    out.FxC_rl.data = squeeze(F_rlCx(1:end-1));
    out.FyC_rl.data = squeeze(F_rlCy(1:end-1));


    out.omega_rr.data = squeeze(velocityWheel_rr(:,1,1:end-1));
    out.power_rr.data = squeeze(velocityWheel_rr(:,1,1:end-1)).*u(:,3);
    out.Fy_rr.data = squeeze(Fy_rr(1:end-1));
    out.Fx_rr.data = squeeze(Fx_rr(1:end-1));
    out.Fz_rr.data = squeeze(Fz_rr(1:end-1));
    out.FxC_rr.data = squeeze(F_rrCx(1:end-1));
    out.FyC_rr.data = squeeze(F_rrCy(1:end-1));

    out.vx.data = v_x(1:end-1);
    out.vy.data = v_y(1:end-1);

    out.ax.data = dz(1,:);
    out.ay.data = dz(2,:);
    out.psi.data = dz(3,:);
    out.dpsi.data = dz(4,:);
    out.beta.data = beta;
    out.X.data = dz(5,:);
    out.Y.data = dz(6,:);

    out.Fz_aero.data = squeeze(Fz_aero);
    out.Fdrag_aero.data = squeeze(F_drag);

    out.alpha_fl.data = squeeze(alpha_fl);
    out.alpha_fr.data = squeeze(alpha_fr);
    out.alpha_rl.data = squeeze(alpha_rl);
    out.alpha_rr.data = squeeze(alpha_rr);

    out.YawTorque.data = squeeze(M);
    
    TvMomemnt = squeeze((-Fx_fl.*cos(delta_f) + Fx_fr.*cos(delta_f) - Fx_rl.*cos(delta_r) + Fx_rr.*cos(delta_r))*car.track/2);
    out.YawMomentTV = TvMomemnt(1:end-1);
    steering = (Fy_fl + Fy_fr).*car.l_f.*cos(delta_f) - (Fy_rl + Fy_rr).*car.l_r.*cos(delta_r);
    out.YawMomentSteering = squeeze(steering(1:end-1));
    
    out.Fbrake.data = MechBrake(1:end-1);
    %% Power from acp

    diffabs = @(x) x.*tanh(x.*0.01);
    
    Power_fl = out.power_fl.data;
    Power_fr = out.power_fr.data;
    Power_rl = out.power_rl.data;
    Power_rr = out.power_rr.data;

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

    out.PowerAcp.data =PowerAcp;
    out.PowerBrake.data = PowerBrake;
    out.t = z(6,1:end);
    out.plotControls

end

end




function out  = mymatmul(a,b)
    if class(a) == "casadi.MX" || class(b) == "casadi.MX" 
        out = a*b;
    else
        out = pagemtimes(a,b);
    end
end

function out = seEl(data,dim) %Select elemetns
    if length(size(data)) == 3
        out = data(:,dim,:);
    else
        out = data(:,dim);
    end


end


