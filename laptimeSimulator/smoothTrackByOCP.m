function [x_traj, y_traj, s_traj, th_traj, C_traj] = smoothTrackByOCP(x_smpl, y_smpl, r, closedTrack, ds)
    

    if nargin < 3
        r = 1e6;
    end
    
    if nargin < 4
        closedTrack = false;
    end
    
    x_smpl = x_smpl(:);
    y_smpl = y_smpl(:);
    
    dx = diff(x_smpl);
    dy = diff(y_smpl);
    ds_tmp = sqrt(dx.^2 + dy.^2);

    s_tmp = [0; cumsum(ds_tmp)];
    
    %%%%%
    N  = round((s_tmp(end)-s_tmp(1))/ds);
    ds = (s_tmp(end)-s_tmp(1))/N;
    s_traj = linspace(s_tmp(1), s_tmp(end), N)';
    x_smpl = interp1(s_tmp, x_smpl, s_traj, 'PCHIP');
    y_smpl = interp1(s_tmp, y_smpl, s_traj, 'PCHIP');    
    %%%%%

    dx = diff(x_smpl);
    dy = diff(y_smpl);
    
    th_init = unwrap(atan2(dy, dx));
    th_init(end+1) = th_init(end);
    C_init = diff(th_init)./ds;
    C_init(end+1) = C_init(end);

    %% %%%%%%%%%%%%%%%
    % Initialize the optimization problem
    opti = casadi.Opti();

    % define decision variables
    Z = opti.variable(N, 4);
    z_C    = Z(:, 1);
    z_th   = Z(:, 2);
    z_x    = Z(:, 3);
    z_y    = Z(:, 4);

    u = opti.variable(N-1, 1);

    % Objective function (minimize the final time)
    x_dev = ((z_x(1:end-1) - x_smpl(1:end-1)).^2 + (z_x(2:end) - x_smpl(2:end)).^2)/2;
    y_dev = ((z_y(1:end-1) - y_smpl(1:end-1)).^2 + (z_y(2:end) - y_smpl(2:end)).^2)/2;

    opti.minimize( sum(ds * ( r*u.^2 + x_dev + y_dev)) );

    % Dynamic constraints
    f = @(z, u) [u;z(1);cos(z(2));sin(z(2))]';

    % ---- RK4 --------
    for k=1:N-1 % loop over control intervals
        dsk = (s_traj(k+1)-s_traj(k));
        % Runge-Kutta 4 integration
        k1 = f(Z(k, :),            u(k, :));
        k2 = f(Z(k, :) + dsk/2*k1, u(k, :));
        k3 = f(Z(k, :) + dsk/2*k2, u(k, :));
        k4 = f(Z(k, :) + dsk*k3,   u(k, :));
        x_next = Z(k, :) + dsk/6*(k1+2*k2+2*k3+k4);
        opti.subject_to(Z(k+1, :)==x_next); % close the gaps
    end

    if closedTrack
        opti.subject_to(z_x(end) == z_x(1))
        opti.subject_to(z_y(end) == z_y(1))
        opti.subject_to(z_th(end) == z_th(1) + 2*pi*round((th_init(end)-th_init(1))/2/pi))
        opti.subject_to(z_C(end) == z_C(1))
    end
    
    opti.set_initial(z_x, x_smpl);
    opti.set_initial(z_y, y_smpl);
    opti.set_initial(z_C, C_init);
    opti.set_initial(z_th, th_init);
    
    opti.set_initial(u, diff(C_init))
    
%     opti.subject_to(-0.05 < u < 0.05)
%     Rmax = 5;
%     opti.subject_to(-1/Rmax < z_C < 1/Rmax)
%     
%     dev_max = 2;
%     opti.subject_to( x_dev < dev_max^2 )
%     opti.subject_to( y_dev < dev_max^2 )
    


    % Solve NLP
    opti.solver('ipopt'); % use IPOPT solver
    sol = opti.solve();
    %%
    z_opt = sol.value(Z)';

    x_traj = z_opt(3, :);
    y_traj = z_opt(4, :);
    C_traj  = z_opt(1, :);
    th_traj  = z_opt(2, :);

