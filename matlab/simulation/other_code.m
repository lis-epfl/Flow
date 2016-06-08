%--------------------------------------------------------------------------
%   OPTIC-FLOW SIMULATOR
%--------------------------------------------------------------------------
% Initialization
clc
clear all;
close all;
seed = 1;
rng(seed);

accuracy = [];

% Camera model
model = struct('pol', [6.660506*10^1, 0.0, -6.426152*10^(-3), 2.306550*10^(-5), -2.726345*10^(-7)],...
		'length_pol', 5, ...
		'invpol', [98.889649, 60.099030, 3.523247, 11.584154, 10.704617, 4.911849, 0.899849],...
		'length_invpol', 7,...
		'xc', 56.232012,...
		'yc', 77.63939272,...
		'c', 1.001183,...
		'd', 0.001337,...
		'e', 0.002268,...
		'width', 160,...
		'height', 120);

for iter = 1:1,
% Generate sampling pixels
N = 80;
size = [model.width, model.height];
p = generate_sampling(size, N);

% Back-project sampling pixels
P = back_project(p, model);
P = normr(P);

% Plot projected optic-flow vectors
% figure;
% hold on
% plot3(P(:,1), P(:,2), P(:,3), '.');
% [x_, y_, z_] = sphere;
% mesh(x_, y_, z_,'Edgecolor', 'k');

% Generate optic-flow
freq = 200;                         % refresh rate of the camera
v = [0.56, 0.42, 0.13]/freq;        % translational velocity of the camera (m.frame^-1)
w = [0, pi/2, pi/4]/freq;           % angular velocity of the camera (rad.frame^-1)
D = 50*10^(-2);                     % depth of the scene
F = generate_optic_flow(v, w, P, D);

% Plot projected optic-flow vectors
% figure;
% hold on
% plot3(P(:,1), P(:,2), P(:,3), '.');
% quiver3(P(:,1), P(:,2), P(:,3), F(:,1), F(:,2), F(:,3));
% [x_, y_, z_] = sphere;
% mesh(x_, y_, z_,'Edgecolor', 'k');

% Add Outliers
pOutliers = 0.25;   % proportion of outliers
% disp(floor(length(P)*pOutliers));
indices = randsample(length(P), floor(length(P)*pOutliers)); % indices of outliers
for i=1:length(indices),
    F(indices(i),:) = F(indices(i),:) + cross(P(indices(i),:),F(indices(i),:))*randn;
end

% Plot optic-flow vectors with outliers
% figure;
% hold on
% plot3(P(:,1), P(:,2), P(:,3), '.');
% quiver3(P(:,1), P(:,2), P(:,3), F(:,1), F(:,2), F(:,3));
% [x_, y_, z_] = sphere;
% mesh(x_, y_, z_,'Edgecolor', 'k');

% Derotate optic-flow
W = repmat(w,length(P),1);
F = F + cross(W,P);

% Plot derotated optic-flow vectors
% figure;
% hold on
% plot3(P(:,1), P(:,2), P(:,3), '.');
% quiver3(P(:,1), P(:,2), P(:,3), F(:,1), F(:,2), F(:,3));
% [x_, y_, z_] = sphere;
% mesh(x_, y_, z_,'Edgecolor', 'k');

% Voting
option = 1; % 1=raw / 2=averaged

% Compute normal vectors
G = get_normal_vector(F,P);

% Get voting bins
load('cbins_big.mat');

c_acc = zeros(length(cbins));   % sin(theta/2)
c_res = 0.26724;

% Perform coarse voting
for i = 1:length(cbins),
    for j = 1:length(G),
        if(abs(G(j,1)*cbins(i,1) + G(j,2)*cbins(i,2) + G(j,3)*cbins(i,3)) < c_res),
            c_acc(i) = c_acc(i)+1;
        end
    end
end

% Find best estimate
d_est = find_best(cbins, c_acc, option);

% Perform refined votings
load('rbins_big.mat');

r_acc = zeros(length(rbins));
r_res = [0.03839]; % sin(theta/2)
bins = rotate_bins(rbins, d_est);

for i = 1:length(bins),
    for j = 1:length(G),
        if(abs(G(j,1)*bins(i,1) + G(j,2)*bins(i,2) + G(j,3)*bins(i,3)) < r_res(1)),
            r_acc(i) = r_acc(i)+1;
        end
    end
end

% Find best estimate
d_est = find_best(bins, r_acc, option);

% Plot results
figure;
hold on
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'FaceAlpha', 0, 'Edgecolor', 'k', 'EdgeAlpha', 0.5);
h1 = quiver3(0, 0, 0, d_est(1), d_est(2), d_est(3));
v = normr(v);
h2 = quiver3(0, 0, 0, v(1), v(2), v(3));
legend([h1,h2], 'estimated direction', 'actual direction');
hold off

% Statistics
accuracy = [accuracy dot(normr(v),d_est)];
end