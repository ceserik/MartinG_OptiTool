function dz = raceCar_time_ODE(z, u, car, t, t_F)
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
delta_f = Ft(1);
delta_r = Ft(2);
Ff      = Ft(3);
Fr      = Ft(4);
Yt      = Ft(5);

% Extract the states from z
v_x  = z(1);
v_y  = z(2);
psi  = z(3);
dpsi = z(4);
X    = z(5);
Y    = z(6);
Fz   = z(7);

% Compute the slip angles
% alpha_f_old = delta_f - atan2(v_y + car.l_f*dpsi, v_x);
alpha_f = -atan2((v_y + car.l_f*dpsi)*cos(delta_f) - v_x*sin(delta_f) ,...
                 (v_y + car.l_f*dpsi)*sin(delta_f) + v_x*cos(delta_f));
% alpha_r_old = delta_r - atan2(v_y - car.l_r*dpsi, v_x);
alpha_r = -atan2((v_y - car.l_r*dpsi)*cos(delta_r) - v_x*sin(delta_r) ,...
                 (v_y - car.l_r*dpsi)*sin(delta_r) + v_x*cos(delta_r));
% [alpha_f_old, alpha_f, alpha_f_old - alpha_f, alpha_r_old, alpha_r, alpha_r_old - alpha_r]
% Compute the forces acting on the wheels

Fz_aero = 0.5*car.airDensity*car.CL*car.A*v_x.^2;

Fz_f = car.m/2 * car.g - Fz - Fz_aero * (1-car.COP);
Fz_r = car.m/2 * car.g + Fz - Fz_aero * car.COP;

F_fy = car.ftire(alpha_f,Fz_f, car);
F_ry = car.ftire(alpha_r,Fz_r, car);
F_fx = Ff;
F_rx = Fr;

% Compute quantities need for the change of the position in the inertial frame
v = sqrt(v_x^2 + v_y^2);
beta = atan2(v_y, v_x);

dvx = 1/car.m  * (F_rx*cos(delta_r) - F_ry*sin(delta_r) + F_fx*cos(delta_f) - F_fy*sin(delta_f) - 0.5*car.airDensity*car.CD*car.A*v_x^2) + dpsi*v_y;
dz = [... 
dvx;                                                                                                                          % v_x/dt
1/car.m  * (F_rx*sin(delta_r) + F_ry*cos(delta_r) + F_fx*sin(delta_f) + F_fy*cos(delta_f)) - dpsi*v_x;                        % v_y/dt  
dpsi;                                                                                                                         % psi/dt
1/car.Izz * (car.l_f*F_fx*sin(delta_f) + car.l_f*F_fy*cos(delta_f) - car.l_r*F_rx*sin(delta_r) - car.l_r*F_ry*cos(delta_r)) %+ Yt/car.Izz;  % dpsi
v*cos(psi + beta);                                                                                                            % X/dt
v*sin(psi + beta);                                                                                                            % Y/dt
(dvx*car.COGz/car.wheelbase*car.m-Fz)/car.Sus_t;                                                                              %Fz/dt  
];

end