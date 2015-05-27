% script to test image mosiac project for CIS581

%% Preprocessing (per images)
verbose = 0;

num_imgs = 3;
ref_im = 2;
non_ref_inds = [1:ref_im-1,ref_im+1:num_imgs];
Imgs = cell(1,num_imgs);
Imgs_Gray = cell(1,num_imgs);
for i=1:num_imgs
    Imgs{i} = double(imread(strcat('img', num2str(i), '.jpg')))/255;
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
ms = cell(1, num_imgs-1);
n = cell(1, num_imgs-1);
x_im = cell(1, num_imgs-1);
y_im = cell(1, num_imgs-1);
x_ref = cell(1, num_imgs-1);
y_ref = cell(1, num_imgs-1);


for i=1:num_imgs-1
    ms{i} = feat_match(ps{non_ref_inds(i)}, ps{ref_im});
    n{i} = ms{i} ~= -1;
    x_im{i} = corner_x{non_ref_inds(i)}(n{i});
    y_im{i} = corner_y{non_ref_inds(i)}(n{i});
    x_ref{i} = corner_x{ref_im}(ms{i}(n{i}));
    y_ref{i} = corner_y{ref_im}(ms{i}(n{i}));
    
    if verbose
        figure();
        showMatchedFeatures(Imgs_Gray{non_ref_inds(i)},Imgs_Gray{ref_im}, [x_im{i},y_im{i}], [x_ref{i}, y_ref{i}], 'montage');
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
verbose=1;debug=1;
ransac_thres = 5;
H = cell(1, num_imgs-1);
inlier_ind = cell(1, num_imgs-1);

for i=1:num_imgs-1
    i
    [H{i}, inlier_ind{i}] = ransac_est_homography(y_im{i},x_im{i}, y_ref{i}, x_ref{i}, ransac_thres);
    %[H23, inlier_ind23] = ransac_est_homography(y3,x3, y22, x22, ransac_thres);
    if verbose
        figure();
        showMatchedFeatures(Imgs_Gray{non_ref_inds(i)},Imgs_Gray{ref_im}, [x_im{i}(inlier_ind{i}),y_im{i}(inlier_ind{i})], [x_ref{i}(inlier_ind{i}), y_ref{i}(inlier_ind{i})], 'montage');
    end
end
%%
% if debug
%     x = 300;
%     y = 300;
%     H=est_homography(x21,y21,x1,y1);
%     [X,Y] = apply_homography(H12,x1,y1);
%     figure(30);
%     subplot(1,2,1);
%     imshow(Imgs{1});
%     hold on;
%     plot(x1,y1, '+r');
%     subplot(1,2,2);
%     imshow(Imgs{2});
%     hold on;
%     plot(X,Y,'+r');
% end
%%



%% Stitch Images Together
tform = cell(1, num_imgs-1);
im_trans = cell(1, num_imgs-1);
R_im = cell(1, num_imgs-1);
R_ref = cell(1, num_imgs-1);
im_warp = cell(1, num_imgs-1);

for i=1:num_imgs-1
    if i==1
        img_mosaic = Imgs{ref_im};
        Rm = imref2d(size(Imgs{ref_im}));
    end
    tform{i} = projective2d(H{i}');
    %tform23 = projective2d(H23');
    [im_trans{i}, R_im{i}] = imwarp(Imgs{non_ref_inds(i)}, imref2d(size(Imgs{non_ref_inds(i)})), tform{i});
    %[im3_t, R3] = imwarp(Imgs{3}, imref2d(size(Imgs{3})), tform23);
    [im_warp{i}, R_ref{i}]= imfuse(im_trans{i}, R_im{i}, Imgs{ref_im}, imref2d(size(Imgs{ref_im})), 'blend');
    %[warp23, R23]= imfuse(im3_t, R3, Imgs{2}, imref2d(size(Imgs{2})), 'blend');
    [img_mosaic, Rm] = imfuse(im_warp{i}, R_im{i}, img_mosaic, Rm, 'blend');
    if verbose
        figure();
        imshow(im_warp{i});
    end
end
    
imwrite(img_mosaic, 'mymosaic.jpg', 'jpg');
