function m = feat_match(p1,p2)

num_imgs = 3;
Imgs = cell(1,num_imgs);
Imgs_Gray = cell(1,num_imgs);
for i=1:num_imgs
    Imgs{i} = double(imread(strcat('im', num2str(i), '.jpg')))/255;
    Imgs_Gray{i} = rgb2gray(Imgs{i});
end

%ssd = @(x,y) sum((x-y).^2);
verbose = 0;
match_ratio_thresh = .6;
m = -ones(size(p1,2),1);


for ii=1:size(p1,2)
    cur_feat = p1(:,ii);
    matches = sum(((cur_feat*ones(1,size(p2,2)))-p2).^2, 1);
    [matches_sorted, inds] = sort(matches);
    best_match = matches_sorted(1);
    second_best = matches_sorted(2);
    match_ratio = best_match/second_best;
    if match_ratio < match_ratio_thresh
        m(ii) = inds(1);
        if verbose
            figure(1);
            subplot(1,2,1);
            imshow(reshape(p1(:, ii), [8,8]));
            axis equal
            subplot(1,2,2);
            imshow(reshape(p2(:, inds(ii)), [8,8]));
            colormap(gray);
            axis equal
            waitforbuttonpress
        end

    end

end
    
        