function plotTrack(track, s, b_plotStartEnd)

if nargin < 3
    b_plotStartEnd = false;
end

hold_state = ishold;

[xc, yc, thc, ~] = track.fcurve(s);
xc_lim1 = -track.w_l.*sin(thc');
xc_lim2 = -track.w_r.*sin(thc');

yc_lim1 =  track.w_l.*cos(thc');
yc_lim2 =  track.w_r.*cos(thc');

hold on
plot(xc, yc, '--', 'Color', 0.5*[1 1 1], 'HandleVisibility','off')
plot(xc_lim1 + xc',   yc_lim1 + yc', 'k', 'LineWidth', 1, 'HandleVisibility','off')
plot(-xc_lim2+ xc',  -yc_lim2 + yc', 'k', 'LineWidth', 1, 'HandleVisibility','off')

if b_plotStartEnd
    plot(xc(1), yc(1), 'ro', 'HandleVisibility','off')
    plot(xc(end), yc(end), 'rx', 'HandleVisibility','off')
end

if ~hold_state
    hold off
end

end

