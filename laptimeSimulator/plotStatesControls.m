function plotStatesControls(z_opt,z_time,u_opt,u_time)
%PLOTSTATESCONTROLS Summary of this function goes here
%   Detailed explanation goes here
states = [esignal; esignal; esignal; esignal; esignal; esignal; esignal];

figure
states(1).data =  z_opt(1:end,1);
states(2).data =  z_opt(1:end,2);
states(3).data =  z_opt(1:end,3);
states(4).data =  z_opt(1:end,4);
states(5).data =  z_opt(1:end,5);
states(6).data =  z_opt(1:end,6);
states(7).data =  z_opt(1:end,7);

states(1).name = "vx_opt";
states(2).name = "vy_opt";
states(3).name = "psi_opt";
states(4).name = "dpsi_opt";
states(5).name = "n_opt";
states(6).name = "t_opt";
states(7).name = "Fz_opt";

eplot(states,z_time)

% plot controls

figure

control =[esignal esignal; esignal esignal;esignal esignal];

control(1,1).data = u_opt(1:end,1);
control(1,2).data = u_opt(1:end,2);
control(2,1).data = u_opt(1:end,3);
control(2,2).data = u_opt(1:end,4);
control(3,1).data = u_opt(1:end,5);

control(1,1).name = "steering front";
control(1,2).name = "steering rear";
control(2,1).name = "Force front?";
control(2,2).name = "Force rear?";
control(3,1).name = "Yaw torque";


eplot(control,u_time)
end

