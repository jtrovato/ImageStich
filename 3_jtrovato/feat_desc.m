function p = feat_desc(I, y, x)

verbose = 0;
G = fspecial('gaussian',[5 5],2);
I_blur = conv2(I, G, 'same');
[h,w] = size(I_blur);

p= zeros(64, size(y, 1));
for ii=1:size(y, 1)
    if y(ii)-20<1 || y(ii)+19 >h || x(ii)-20 < 1 || x(ii)+19 > w %dont include points that are one the edges
        continue
    end
    
    des40 = I_blur((y(ii)-20):1:(y(ii)+19), (x(ii)-20):1:(x(ii)+19));
    %des40 = rot90(des40, -1);
    des8 = des40(1:5:end,1:5:end);
    des8 = (des8 - mean2(des8))/std2(des8); %normalize the descriptor
    p(:,ii) = reshape(des8, [64,1]);
    
    if verbose
        subplot(1,2,1);
        imshow(I_blur);
        colormap(gray);
        hold on
        plot(x(ii),y(ii), 'rx')
        subplot(1,2,2);
        imshow(reshape(des8, [8,8]));
        colormap(gray);
        axis equal
        waitforbuttonpress
    end
        
end
    

