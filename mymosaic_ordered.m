function img_mosaic = mymosaic_ordered(img_input)
% img_input is a cell array of input images
verbose = 1;
Imgs = img_input;
num_imgs = 3;
ref_im = 2;
non_ref_inds = [1:ref_im-1,ref_im+1:num_imgs];
%% convert to grayscale
Imgs_Gray = cell(1,num_imgs);
for i=1:num_imgs
    Imgs_Gray{i} = rgb2gray(Imgs{i});
end

%% Corner Detection (per image)
Corners = cell(1,num_imgs);
for i=1:num_imgs
    Corners{i} = cornermetric(Imgs_Gray{i}, 'Harris');
end

%% Adaptive Non-Maximum Supression (per image)
num_corners = 100;
corner_x = cell(1,num_imgs);
corner_y = cell(1,num_imgs);
for i=1:num_imgs
    [corner_y{i}, corner_x{i}, rmax] = anms(Corners{i}, num_corners);
end

%% Extract Feature Descriptors (per image)
ps = cell(1,num_imgs);
for i=1:num_imgs
    ps{i} = feat_desc(Imgs_Gray{i}, corner_y{i}, corner_x{i});
end

%% Matching Descriptors (per match)
% assuming imaages come in order.
% ref refers tothe image after the current image.
ms = cell(1, num_imgs-1);
n = cell(1, num_imgs-1);
x_im = cell(1, num_imgs-1);
y_im = cell(1, num_imgs-1);
x_ref = cell(1, num_imgs-1);
y_ref = cell(1, num_imgs-1);


for i=1:num_imgs-1
    ms{i} = feat_match(ps{i}, ps{i+1});
    n{i} = ms{i} ~= -1;
    x_im{i} = corner_x{i}(n{i});
    y_im{i} = corner_y{i}(n{i});
    x_ref{i} = corner_x{i+1}(ms{i}(n{i}));
    y_ref{i} = corner_y{i+1}(ms{i}(n{i}));
    
    if verbose
        figure();
        showMatchedFeatures(Imgs_Gray{i},Imgs_Gray{i+1}, [x_im{i},y_im{i}], [x_ref{i}, y_ref{i}], 'montage');
    end

end


% ms{1} = feat_match(ps{1}, ps{2});
% n1 = ms{1} ~= -1;
% x1 = corner_x{1}(n1);
% y1 = corner_y{1}(n1);
% x21 = corner_x{2}(ms{1}(n1));
% y21 = corner_y{2}(ms{1}(n1));
% 
%     if verbose
%         figure();
%         showMatchedFeatures(Imgs_Gray{1},Imgs_Gray{ref_im}, [x1,y1], [x21, y21], 'montage');
%     end
%ms{2} = feat_match(ps{3}, ps{2});
% n2 = ms{2} ~= -1;
% x3 = corner_x{3}(n2);
% y3 = corner_y{3}(n2);
% x22 = corner_x{2}(ms{2}(n2));
% y22 = corner_y{2}(ms{2}(n2));

% % code to check individual matches
% if verbose
%     for ii = 1:size(ms{1},1)
%         %plot both features
%         if(ms{1}(ii) ~= -1)
%             figure(1);
%             subplot(1,2,1);
%             imshow(reshape(ps{1}(:, ii), [8,8]));
%             axis equal
%             subplot(1,2,2);
%             imshow(reshape(ps{2}(:, ms{1}(ii)), [8,8]));
%             colormap(gray);
%             axis equal
%             figure(2)
%             showMatchedFeatures(Imgs_Gray{1},Imgs_Gray{2}, [corner_x{1}(ii), corner_y{1}(ii)], [corner_x{2}(ms{1}(ii)), corner_y{2}(ms{1}(ii))], 'montage');
%             waitforbuttonpress
%         end
%     end
% end



%% Estimate Homography with RANSAC
verbose=1;debug=0;
ransac_thres = 5;
H = cell(1, num_imgs-1);
inlier_ind = cell(1, num_imgs-1);

for i=1:num_imgs-1
    [H{i}, inlier_ind{i}] = ransac_est_homography(y_im{i},x_im{i}, y_ref{i}, x_ref{i}, ransac_thres);
    %[H23, inlier_ind23] = ransac_est_homography(y3,x3, y22, x22, ransac_thres);
    if verbose
        figure();
        showMatchedFeatures(Imgs_Gray{i},Imgs_Gray{i+1}, [x_im{i}(inlier_ind{i}),y_im{i}(inlier_ind{i})], [x_ref{i}(inlier_ind{i}), y_ref{i}(inlier_ind{i})], 'montage');
    end
end
%
if debug
    i = 1;
    x = 300;
    y = 300;
    [X,Y] = apply_homography(H{i},x_im{i},y_im{i});
    figure(30);
    subplot(1,2,1);
    imshow(Imgs{i});
    hold on;
    plot(x1,y1, '+r');
    subplot(1,2,2);
    imshow(Imgs{i+1});
    hold on;
    plot(X,Y,'+r');
end
%



%% Stitch Images Together
verbose = 1;
tform = cell(1, num_imgs-1);
im_trans = cell(1, num_imgs-1);
R_im = cell(1, num_imgs);
R_ref = cell(1, num_imgs-1);
im_warp = cell(1, num_imgs-1);

for i=1:num_imgs
    
    
    tform{i-1} = projective2d(H{i-1}');
    %tform23 = projective2d(H23');
    [im_trans{i-1}, R_im{i-1}] = imwarp(Imgs{i-1}, imref2d(size(Imgs{i-1})), tform{i-1});
    %[im3_t, R3] = imwarp(Imgs{3}, imref2d(size(Imgs{3})), tform23);
    %[im_warp{i}, R_ref{i}]= imfuse(im_trans{i}, R_im{i}, Imgs{ref_im}, imref2d(size(Imgs{ref_im})), 'blend');
    %[warp23, R23]= imfuse(im3_t, R3, Imgs{2}, imref2d(size(Imgs{2})), 'blend');
    if i==1
        img_mosaic = im_trans{i-1};
        Rm = R_im{i-1};
        continue;
    end
    [img_mosaic, Rm] = imfuse(im_trans{i-1}, R_im{i-1}, Imgs{i}, imref2d(size(Imgs{i})), 'blend');
    
    if verbose
        figure();
        imshow(im_trans{i-1});
        figure();
        imshow(img_mosaic);
    end
end
    
imwrite(img_mosaic, 'mymosaic.jpg', 'jpg');