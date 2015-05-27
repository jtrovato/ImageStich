% script to test image mosiac project for CIS581
%given a series of images each images matches with the next iamge in the
%series

%% Preprocessing (per images)
verbose = 0;

num_imgs = 3;
ref_im = 2;
non_ref_inds = [1:ref_im-1,ref_im+1:num_imgs];
Imgs = cell(1,num_imgs);
for i=1:num_imgs
    Imgs{i} = double(imread(strcat('img', num2str(i), '.jpg')))/255;
end

img_mosaic = mymosaic(Imgs);
imshow(img_mosaic);