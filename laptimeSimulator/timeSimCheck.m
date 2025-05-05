function [tsim,zsim] = timeSimCheck(expdata)
%TIMESIMCHECK Summary of this function goes here
%   Detailed explanation goes here

z_opt     = expdata.z;
u_opt     = expdata.u;
t_opt     = expdata.z(:,6); % time vector
Ft_opt    = t_opt(1:end-1);
x_car_opt = expdata.carpos(:,1);
y_car_opt = expdata.carpos(:,2);

car = expdata.car;


%opts = odeset('reltol',1.e-6,'AbsTol',1.e-6);
opts = odeset('reltol',1.e-12,'AbsTol',1.e-12,'OutputFcn',@odeplot);
opts = odeset('reltol',1.e-11,'AbsTol',1.e-11);
%figure(67)
clf

u_opt(1:end,3) = u_opt(1:end,3) ;%+  u_opt(1:end,8)/2*car.BrakeBalance;
u_opt(1:end,4) = u_opt(1:end,4) ;%+  u_opt(1:end,8)/2*car.BrakeBalance;
u_opt(1:end,5) = u_opt(1:end,5) ;%+  u_opt(1:end,8)/2*(1-car.BrakeBalance);
u_opt(1:end,6) = u_opt(1:end,6) ;%+  u_opt(1:end,8)/2*(1-car.BrakeBalance);

if expdata.car.tracks ==2
    [t_sim, z_sim] = ode15s(@(t, z) twinRaceCar_time_ODE(z, u_opt, car, t, Ft_opt), [t_opt(1) t_opt(end)], [z_opt(1, 1:4) x_car_opt(1) y_car_opt(1) z_opt(1, 7 ) z_opt(1, 8)],opts);

else
    [t_sim, z_sim] = ode113(@(t, z) raceCar_time_ODE(z, u_opt, car, t, Ft_opt), [t_opt(1) t_opt(end)], [z_opt(1, 1:4) x_car_opt(1) y_car_opt(1) z_opt(1, 7)],opts);
end



%plotStatesControls(z_sim,t_sim,u_opt,Ft_opt);


states = [
    esignal esignal;
    esignal esignal;
    esignal esignal;
    esignal esignal;
    esignal esignal;];


states(1,1).data = z_sim(:,1);
states(1,1).time = t_sim;
states(1,2).data = z_opt(:,1);
states(1,2).time = t_opt;

states(2,1).data = z_sim(:,2);
states(2,1).time = t_sim;
states(2,2).data = z_opt(:,2);
states(2,2).time = t_opt;

states(3,1).data = z_sim(:,3);
states(3,1).time = t_sim;
states(3,2).data = z_opt(:,3);
states(3,2).time = t_opt;

states(4,1).data = z_sim(:,4);
states(4,1).time = t_sim;
states(4,2).data = z_opt(:,4);
states(4,2).time = t_opt;

states(5,1).data = z_sim(:,5);
states(5,1).time = t_sim;
states(5,2).data = z_opt(:,5);
states(5,2).time = t_opt;

states(6,1).data = z_sim(:,7);
states(6,1).time = t_sim;
states(6,2).data = z_opt(:,7);
states(6,2).time = t_opt;



states(1,1).name = "vx_sim";
states(1,2).name = "vx_opt";
states(2,1).name = "vy_sim";
states(2,2).name = "vy_opt";
states(3,1).name = "psi_sim";
states(3,2).name = "psi_opt";
states(4,1).name = "Dpsi_sim";
states(4,2).name = "Dpsi_opt";
states(5,1).name = "n_sim";
states(5,2).name = "n_opt";
states(6,1).name = "Fz_sim";
states(6,2).name = "Fz_opt";

hold off
figure(68)
eplot(states,1)

hold off
figure(69)
clf
x_car_sim = z_sim(:,5);
y_car_sim = z_sim(:,6);

hold on

plot(x_car_opt, y_car_opt, '-x')
plot(x_car_sim, y_car_sim, '--')

legend('Optimal', 'Simulated')
hold off
axis equal
grid on

%vx_opt    = z_opt(:,1); % longitudinal velocity of the car
%vy_opt    = z_opt(:,2); % lateral velocity of the car
%psi_opt   = z_opt(:,3); % orientation of the car
%Dpsi_opt  = z_opt(:,4); % angular velocity of the car
%n_opt     = z_opt(:,5); % perpendicular position of the car with respect to the track center line
%t_opt     = z_opt(:,6); % time vector
%Fz_opt    = z_opt(:,7); % Fz vector
%Ft_opt    = t_opt(1:end-1); % time vector for the controls - this is just the time vector t_opt shorter by one element as we the controls are not applied at the end of the last control period (we have N values for each control input and N+1 values for each state)




end

