function [H, inlier_ind] = ransac_est_homography(y1, x1, y2, x2, thres)
% Function to compute homography between matched features from two imagess.
% The algorithm randomly selects matches computes the homographies and votes
% for the most common homography.
%
% the point parameters are all Nx1 matrices
% H is a 3x3 homography matrix
% inlier_ind - indexes of the inliers
num_iters = 1000;
homographies = cell(num_iters, 1);
votes = zeros(num_iters, 1);
for ii=1:num_iters
    %randomly select num_rand_matches matchess
    num_matches = size(y1,1);
    num_rand_matches = 4;
    rand_inds = randsample(num_matches, num_rand_matches);
    other_inds = setdiff(1:num_matches, rand_inds);

    ry1 = y1(rand_inds);
    rx1 = x1(rand_inds);
    ry2 = y2(rand_inds);
    rx2 = x2(rand_inds);

    oy1 = y1(other_inds);
    ox1 = x1(other_inds);
    oy2 = y2(other_inds);
    ox2 = x2(other_inds);

    %compute homography of random points
    curH = est_homography(rx2, ry2, rx1, ry1);
    homographies{ii} = curH;
    % apply this homogrphy to all the other points
    [hx, hy] = apply_homography(curH, ox1, oy1);
    %check the error of other points with guessed homography
    ssd = sum((([ox2,oy2]-[hx,hy]).^2), 2); %compute sum of squared differeneces (sqaure of distance)

    inliers = find(ssd<thres);
    num_inliers = size(inliers, 1);
    votes(ii) = num_inliers;
end

size(x1)
[max_votes, best_ind] = max(votes)

H = homographies{best_ind};
% apply this homogrphy to all the other points
[hx, hy] = apply_homography(H, x1, y1);
%check the error of other points with guessed homography
ssd = sum((([x2,y2]-[hx,hy]).^2), 2); %compute sum of squared differeneces (sqaure of distance)

inlier_ind = find(ssd<thres);
inliers_x1 = x1(inlier_ind);
inliers_y1 = y1(inlier_ind);
inliers_x2 = x2(inlier_ind);
inliers_y2 = y2(inlier_ind);

H = est_homography(inliers_x2, inliers_y2, inliers_x1, inliers_y1);






