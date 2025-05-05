function plotTrack(track, s, b_plotStartEnd,ax)

if nargin < 3
    b_plotStartEnd = false;
    ax = gca;
end

if ax ==0
    ax = gca;
end
hold_state = ishold(ax);

[xc, yc, thc, ~] = track.fcurve(s);
xc_lim1 = -track.w_l.*sin(thc');
xc_lim2 = -track.w_r.*sin(thc');

yc_lim1 =  track.w_l.*cos(thc');
yc_lim2 =  track.w_r.*cos(thc');

hold(ax,'on') 
plot(ax,xc, yc, '--', 'Color', 0.5*[1 1 1], 'HandleVisibility','off')
plot(ax,xc_lim1 + xc',   yc_lim1 + yc', 'k', 'LineWidth', 1, 'HandleVisibility','off')
plot(ax,-xc_lim2+ xc',  -yc_lim2 + yc', 'k', 'LineWidth', 1, 'HandleVisibility','off')

if b_plotStartEnd
    plot(ax,xc(1), yc(1), 'ro', 'HandleVisibility','off')
    plot(ax,xc(end), yc(end), 'rx', 'HandleVisibility','off')
end

if ~hold_state
   hold(ax,'off') 
end

end

