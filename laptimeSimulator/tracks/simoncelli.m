img = imread('Simoncelli.png');
img = rgb2gray(img) < 140;

%%
img_track = img;

figure(2)
[C, h] = imcontour(img_track);
%%
nxy = C(2,1);

skip = 5;

x_smpl = C(1,2:skip:nxy);
y_smpl = C(2,2:skip:nxy);

plot(x_smpl, y_smpl, '.')

axis equal

save simoncelli x_smpl y_smpl
