function [dzds hiddenStates] = twinRaceCar_path_ODE(s, z, u, car, track,expdata)
% s - arc length along the curve
% z = [vx; vy; psi; dpsidt; n; t], vx-longitudinal velocity, vy-lateral velocity, psi-angular position of the car with respect to the inertial frame, n-perpendicular distance to the curve, t-time
% u = [delta_f; delta_r; Ff; Fr], delta_f - steering angle of the front wheel, delta_r - steering angle of the rear wheel, Ff - force acting on the front wheel in the longitudinal direction, Fr - force acting on the rear wheel in the longitudinal direction
% car - structure containing parameters of the car model
% track - structure containing:
%          [xc, yc, th, C] = fcurve(s) - th=angle of the curve, C=curviture of the curve
%          width = width of the track in meters


A = uint8.empty;
if nargin > 5
    [dzdt hiddenStates] = twinRaceCar_time_ODE(z', u', car,A,A,expdata);
else
    [dzdt hiddenStates] = twinRaceCar_time_ODE(z', u', car,A,A);
end
[~, ~, th, C] = track.fcurve(s);

v_x     = z(1,:);
v_y     = z(2,:);
psi     = z(3,:);
n       = z(5,:);

epsilon = psi - th';

Sf = (1 - n*C) ./ (v_x.*cos(epsilon) - v_y.*sin(epsilon));
dndt = v_x.*sin(epsilon) + v_y.*cos(epsilon);

dzds = [... 
    Sf .* dzdt(1,:);
    Sf .* dzdt(2,:);
    Sf .* dzdt(3,:);
    Sf .* dzdt(4,:);
    Sf .* dndt;
    Sf;
    Sf .* dzdt(7,:);
    Sf .* dzdt(8,:);];
end

