img = imread('479386-blackwood_gptrack.png');
img = rgb2gray(img)> 200;

img_close = imclose(img, strel('sphere', 6));
img_erode = imerode(img_close, strel('sphere', 4));

figure(1)
subplot(131)
imshow(img)

subplot(132)
imshow(img_close)

subplot(133)
imshow(img_erode)


%%
img_track = img_close;

figure(2)
[C, h] = imcontour(img_erode);
%%
nxy = C(2,1);

skip = 5;

x_smpl = C(1,2:skip:nxy);
y_smpl = C(2,2:skip:nxy);

plot(x_smpl, y_smpl, '.')

axis equal

save blackwood x_smpl y_smpl
