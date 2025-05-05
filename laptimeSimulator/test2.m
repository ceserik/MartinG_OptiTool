% Create a transform object
t = hgtransform;

% Create a plot and set its parent to the transform
h = plot(x, y, 'Parent', t);

% Define a rotation matrix
theta = linspace(0, 2*pi, 100);
for k = 1:length(theta)
    % Create a rotation matrix
    R = makehgtform('zrotate', theta(k));
    % Apply the transformation
    set(t, 'Matrix', R);
    drawnow;
end
