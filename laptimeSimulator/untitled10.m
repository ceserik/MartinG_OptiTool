% Torque values for each wheel [Nm]
torque = [120, 135, 130, 140];  % [FL, FR, RL, RR]

% Create a new figure and axes
figure;
ax = axes;
hold on;
axis equal;
axis off;
title('Wheel Torque Distribution');

% Define bar width and spacing
barWidth = 0.3;
maxTorque = max(torque);

% Positions for each wheel (x, y)
positions = [
    1, 2;  % FL
    2, 2;  % FR
    1, 1;  % RL
    2, 1   % RR
];

labels = {'FL', 'FR', 'RL', 'RR'};

% Draw each torque bar at correct position
for i = 1:4
    x = positions(i,1);
    y = positions(i,2);

    % Plot vertical bar upward from wheel center
    rectangle('Position', [x - barWidth/2, y, barWidth, torque(i)/maxTorque],'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
    
    % Add text label
    text(x, y - 0.2, labels{i}, 'HorizontalAlignment', 'center', 'FontSize', 12);
    text(x, y + torque(i)/maxTorque + 0.1, sprintf('%d Nm', torque(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 10);
end

% Set limits
xlim([0.5 2.5]);
ylim([0.5 3.5]);
