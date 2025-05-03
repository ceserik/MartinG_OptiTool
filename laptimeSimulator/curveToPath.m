function [x, y, s, th, C] = curveToPath(x_smpl, y_smpl)

    dx = diff(x_smpl(:));
    dy = diff(y_smpl(:));
    ds = sqrt(dx.^2 + dy.^2);

    s = [0; cumsum(ds)];
    %%
    x_smpl = interp1(s, x_smpl, linspace(s(1), s(end), 5e3), 'PCHIP');
    y_smpl = interp1(s, y_smpl, linspace(s(1), s(end), 5e3), 'PCHIP');
    
    dx = diff(x_smpl(:));
    dy = diff(y_smpl(:));
    ds = sqrt(dx.^2 + dy.^2);

    s = [0; cumsum(ds(1:end-1))];
    
    th = unwrap(atan2(dy, dx));
    C = diff(th)./ds(1:end-1);
    C(end+1) = C(end);

    x = x_smpl(1:end-1);
    y = y_smpl(1:end-1);
end

