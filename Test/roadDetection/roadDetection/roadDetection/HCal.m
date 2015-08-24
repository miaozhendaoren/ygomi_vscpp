clc;
clear all;close all;
clc;

% H = textread('H.txt');
% H = inv(H');

%% Horizon
% pointOriginalImage = [];
% 
% for i = 200 : 1 : 480
%     pointOriginalImage = [pointOriginalImage;50 i 1];
% end
% 
% middle = pointOriginalImage*H;
% 
% pointBirdView = [];
% 
% for i = 1 : size(middle, 1)
%     pointBirdView = [pointBirdView;middle(i, 1)/middle(i, 3) middle(i, 2)/middle(i, 3)];
% end

%% test
% pointOriginalImage = [0 359 1];
% 
% middle = pointOriginalImage*H;
% 
% pointBirdView = [middle(1)/middle(3) middle(2)/middle(3)];
% pointBirdView

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% cal invert H by OpenCV
% invertH = textread('invertH.txt');
% 
% pointOriginalImage = [479 329 1];
% 
% middle = pointOriginalImage*invertH;
% 
% pointBirdView = [middle(1)/middle(3) middle(2)/middle(3)];
% pointBirdView