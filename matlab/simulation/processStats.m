%--------------------------------------------------------------------------
% PROCESS STATS
%--------------------------------------------------------------------------
clc

% Accuracy: 2 stages, pOutliers = 0.25
load('acc_2_0.25.mat');
m = mean(accuracy);
s = std(accuracy);
fprintf('Accuracy: 2 stages, pOutliers = 0.25 \n Mean: %f \n Std. dev.: %f \n', m, s);  
clear all

% Accuracy: 2 stages, pOutliers = 0.50
load('acc_2_0.5.mat');
m = mean(accuracy);
s = std(accuracy);
fprintf('Accuracy: 2 stages, pOutliers = 0.5 \n Mean: %f \n Std. dev.: %f \n', m, s);  
clear all

% Accuracy: 2 stages, pOutliers = 0.75
load('acc_2_0.75.mat');
m = mean(accuracy);
s = std(accuracy);
fprintf('Accuracy: 2 stages, pOutliers = 0.75 \n Mean: %f \n Std. dev.: %f \n', m, s);  
clear all

% Accuracy: 5 stages, pOutliers = 0.25
load('acc_5_0.25.mat');
m = mean(accuracy);
s = std(accuracy);
fprintf('Accuracy: 5 stages, pOutliers = 0.25 \n Mean: %f \n Std. dev.: %f \n', m, s);  
clear all

% Accuracy: 5 stages, pOutliers = 0.50
load('acc_5_0.5.mat');
m = mean(accuracy);
s = std(accuracy);
fprintf('Accuracy: 5 stages, pOutliers = 0.5 \n Mean: %f \n Std. dev.: %f \n', m, s);  
clear all

% Accuracy: 5 stages, pOutliers = 0.75
load('acc_5_0.75.mat');
m = mean(accuracy);
s = std(accuracy);
fprintf('Accuracy: 5 stages, pOutliers = 0.75 \n Mean: %f \n Std. dev.: %f \n', m, s);  
clear all