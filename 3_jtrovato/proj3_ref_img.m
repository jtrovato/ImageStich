% script to test image mosiac project for CIS581
%given a series of images and a reference image all images match with
clear all; close all;
%% Preprocessing (per images)
verbose = 0;
num_imgs = 3;
Imgs = cell(1,num_imgs);
for i=1:num_imgs
    Imgs{i} = double(imread(strcat('img', num2str(i), '.jpg')))/255;
end

img_mosaic = mymosaic(Imgs);
imshow(img_mosaic);