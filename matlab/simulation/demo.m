%--------------------------------------------------------------------------
%   OPTIC-FLOW SIMULATOR
%--------------------------------------------------------------------------
% Initialization
clc
clear all;
close all;
seed = 1;
rng(seed);

accuracy =[];

% Camera model
model = struct('pol', [6.660506*10^1, 0.0, -6.426152*10^(-3), 2.306550*10^(-5), -2.726345*10^(-7)],...
		'length_pol', 5, ...
		'invpol', [98.889649, 60.099030, 3.523247, 11.584154, 10.704617, 4.911849, 0.899849],...
		'length_invpol', 7,...
		'xc', 56.232012,...
		'yc', 78.63939,...
		'c', 1.001183,...
		'd', 0.001337,...
		'e', 0.002268,...
		'width', 160,...
		'height', 120);

% Generate sampling pixels
N = 10;
size = [model.width, model.height];
p = generate_sampling(size, N);

figure;
hold on
plot(p(:,2), p(:,1), '.', 'MarkerSize', 8);
[x_, y_, z_] = sphere;
xlabel('Width (px)')
ylabel('Height (px)')
set(gca, 'FontSize', 12)
ax = gca;
ax.XTick = 0:1:160;
ax.YTick = 0:1:120;
set(gca,'xticklabel',[])
set(gca,'yticklabel',[])
grid on

%% Back-project sampling pixels
P = back_project(p, model);
P = normr(P);

figure;
hold on
plot3(P(:,1), P(:,2), P(:,3), '.', 'MarkerSize', 14);
xlabel('X');
ylabel('Y');
zlabel('Z');
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'Edgecolor', 'k', 'FaceAlpha', 0.4, 'EdgeAlpha', 0.1, 'LineWidth', 2);
set(gca, 'FontSize', 12)
grid on

%% Plot projected optic-flow vectors

% Generate optic-flow
freq = 200;                         % refresh rate of the camera
v = [0.5, 0, 0]/freq;        % translational velocity of the camera (m.frame^-1)
w = [0, 0, 0]/freq;           % angular velocity of the camera (rad.frame^-1)
D = 50*10^(-2);                     % depth of the scene
F = generate_optic_flow(v, w, P, D);

% Plot projected optic-flow vectors
figure;
hold on
h1 = plot3(P([2,5,8],1), P([2,5,8],2), P([2,5,8],3), '.', 'MarkerSize', 14);
h2 = quiver3(P([2,5,8],1), P([2,5,8],2), P([2,5,8],3), F([2,5,8],1), F([2,5,8],2), F([2,5,8],3), 'LineWidth', 2);
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'Edgecolor', 'k', 'EdgeAlpha', 0.1, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
legend([h1, h2], 'Sampling points', 'Optic-flow vectors');
set(gca, 'FontSize', 12)
grid on

%% Principle of the normal axis
% Compute normal vectors
G = get_normal_vector(F,P);
G = normr(G);

% Plot great circle and normal vectors
figure;
hold on
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'Edgecolor', 'k', 'FaceColor', 'none', 'EdgeAlpha', 0.1, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
ax = gca;
ax.XTick = -2:0.5:2;
ax.YTick = -2:0.5:2;
ax.ZTick = -2:0.5:2;
for n = 1:length(t),
    v(n,:) = cos(t(n))*F(1,:) + sin(t(n))*P(1,:);
end
h1 = quiver3(0, 0, 0, P(1,1), P(1,2), P(1,3), 'LineWidth', 2);
h2 = quiver3(P(1,1), P(1,2), P(1,3), F(1,1), F(1,2), F(1,3), 'LineWidth', 2);
h3 = quiver3(0, 0, 0, -G(1,1), -G(1,2), -G(1,3), 'LineWidth', 2);
h4 = plot3(v(:,1), v(:,2), v(:,3), 'LineWidth', 2);
legend([h1 h2 h3 h4], 'Sampling direction', 'Optic-flow vector', 'Normal vector', 'Great circle');
grid on

%% Voting
% Compute normal vectors
G = get_normal_vector(F,P);
G = normr(G);

% Draw great circles:
t = [0:pi/100:2*pi];
F = normr(F);

figure;
hold on
[x_, y_, z_] = sphere;
mesh(x_, y_, z_, 'Edgecolor', 'k', 'EdgeAlpha', 0.1, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
ax = gca;
ax.XTick = -2:0.5:2;
ax.YTick = -2:0.5:2;
ax.ZTick = -2:0.5:2;
% legend([h1, h2], 'Sampling points', 'Optic-flow vectors');
grid on
v = zeros(length(t),3);
h = zeros(length(F),1);
for i=2:3:8,
    for n = 1:length(t),
        v(n,:) = cos(t(n))*F(i,:) + sin(t(n))*P(i,:);
    end
    h(i) = plot3(v(:,1), v(:,2), v(:,3), 'LineWidth', 2);
end
h4 = quiver3(1, 0, 0, 1, 0, 0, 'LineWidth', 2);
h5 = quiver3(P(2,1), P(2,2), P(2,3), F(2,1), F(2,2), F(2,3), 'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
h6 = quiver3(P(5,1), P(5,2), P(5,3), F(5,1), F(5,2), F(5,3), 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]);
h7 = quiver3(P(8,1), P(8,2), P(8,3), F(8,1), F(8,2), F(8,3), 'LineWidth', 2, 'Color', [0.9290 0.6940 0.1250]);
set(gca, 'FontSize', 12)
legend([h5, h(2), h6, h(5), h7, h(8), h4], 'Optic-flow n°1', 'Great circle n°1', 'Optic-flow n°2', 'Great circle n°2', 'Optic-flow n°3', 'Great circle n°3', 'Direction of motion');

%% Coarse voting bins
option = 1; % 1=raw / 2=averaged
close all;
% Get voting bins
load('cbins_big.mat');

% Plot coarse bins
figure;
hold on
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'Edgecolor', 'k', 'FaceColor', 'none', 'EdgeAlpha', 0.1, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
ax = gca;
plot3(cbins(:,1), cbins(:,2), cbins(:,3), '.', 'MarkerSize', 12);
set(gca, 'FontSize', 12);
grid on;

%% Perform coarse voting procedure
c_acc = zeros(length(cbins));   % sin(theta/2)
c_res = 0.76605;

% Perform coarse voting
for i = 1:length(cbins),
    for j = 1:length(G),
        if(abs(G(j,1)*cbins(i,1) + G(j,2)*cbins(i,2) + G(j,3)*cbins(i,3)) < c_res),
            c_acc(i) = c_acc(i)+1;
        end
    end
end

%% Refined voting bins before rotation

% Perform refined votings
load('rbins_big.mat');

figure;
hold on
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'Edgecolor', 'k', 'FaceColor', 'none', 'EdgeAlpha', 0.1, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
ax = gca;
plot3(rbins(:,1), rbins(:,2), rbins(:,3), '.', 'MarkerSize', 12);
set(gca, 'FontSize', 12);
grid on;

%% Rotate refined bins
bins = rbins;
bins = rotate_bins(bins, [1,0,0]);

figure;
hold on
[x_, y_, z_] = sphere;
mesh(x_, y_, z_,'Edgecolor', 'k', 'FaceColor', 'none', 'EdgeAlpha', 0.1, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
ax = gca;
h1 = quiver3(0,0,0, 1, 0, 0, 'LineWidth', 2);
h2 = plot3(bins(:,1), bins(:,2), bins(:,3), '.', 'MarkerSize', 12);
set(gca, 'FontSize', 12);
legend([h1, h2], 'Direction estimate', 'Rotated refined bins');
grid on;

%% Perform refined voting

r_acc = zeros(length(rbins1));
r_res = [0.03839]; % sin(theta/2)

for k = 0:3,
    bins = rbins(:,1+3*k:3*(k+1));
    bins = rotate_bins(bins, d_est);
    for i = 1:length(bins),
        for j = 1:length(G),
            if(abs(G(j,1)*bins(i,1) + G(j,2)*bins(i,2) + G(j,3)*bins(i,3)) < r_res(k+1)),
                r_acc(i) = r_acc(i)+1;
            end
        end
    end
    d_est = find_best(bins, r_acc, option);
%     disp(dot(normr(v),d_est));
    r_acc = zeros(length(r_acc));
end

