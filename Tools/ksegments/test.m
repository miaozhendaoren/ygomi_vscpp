
% clear all;
close all;
clc;

for idx = 36
    S = load(['E:\Newco\demo\code\newco_demo04\Demo\Ford\server\server\log\S' num2str(idx) '.txt']);
    X = load(['E:\Newco\demo\code\newco_demo04\Demo\Ford\server\server\log\X' num2str(idx) '.txt']);
    
%     [S_mat] = k_seg_soft(X(:,:),14,2*0.03^2,1);
% 
%     S_mat = S_mat ./ 1200;
    
    figure;
    plot(X(:,1) ./ 1200, X(:,2)./ 1200, 'b*');
    hold on;
    plot(S(:,1), S(:,2), 'ro-');
    hold on;
%     plot(S_mat(:,1), S_mat(:,2),'g^-');
    hold off;
end

% X=get_data(300,0.03,'cros3');
% k_seg_soft(X,14,2*0.03^2,1,1);

% load X.mat;
% X = X .* 300;

step = 300;
idx = step;
overlap = 15;
while idx <= size(X, 1)
    startIdx = idx - step + 1 - overlap;
    if startIdx <= 0
        startIdx = 1;
    end
    
    [S_mat] = k_seg_soft(X(:,:),14,2*0.03^2,1);
    idx = idx + step;

    S_mat = S_mat ./ 300;
    
    figure(10);
    hold on;
    plot(S_mat(:,1), S_mat(:,2),'g^-');
    hold off;
end
