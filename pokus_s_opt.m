function s = pokus_s_opt(s_opt, C_track, ds_min,ds_max)
   % ds_min = 0.1; ds_max = 0.3;
    s = 0;

    while s(end) < s_opt(end)
        Ck = abs(interp1(s_opt, C_track, s(end)));
%         s(end+1) = s(end) + (1 - min(Ck, 1))*(ds_max - ds_min) + ds_min;
        s(end+1) = s(end) + (1 - sqrt(min(Ck, 1)))*(ds_max - ds_min) + ds_min;
    end
    
    s(end) = s_opt(end);
end