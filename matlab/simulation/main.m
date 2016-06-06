%--------------------------------------------------------------------------
%   OPTIC-FLOW SIMULATOR
%--------------------------------------------------------------------------
clear all;

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

% Generate sampling pixels
N = 80;
size = [model.width, model.height];
p = generate_sampling(size, N);

% Back-project sampling pixels
P = back_project(p, model);
P = normr(P);

% Generate optic-flow
v = [0.10, 0.20, 0.05];
w = [0, pi/2, pi/4];
D = 30*10^(-2);
F = generate_optic_flow(v, w, P, D);

% Add noise & Outliers
%--------------------------------------------------------------------------
% TODO
%--------------------------------------------------------------------------

% Compute normal vectors
G = get_normal_vector(F,P);

% Get voting bins
cbins = load('cbins.mat');

c_acc = zeros(size(cbins,1));   % coarse precision (sin(31 degrees/2))
c_res = 0.76605;

% Perform coarse voting
for i = 1:size(cbins,1),
    for j = 1:size(G,1),
        if(abs(G(j,1)*cbins(i,1) + G(j,2)*cbins(i,2) + G(j,3)*cbins(i,3) < c_res)),
            c_acc(i) = c_acc(i)+1;
        end
    end
end

% Find best estimate
d_est = find_best(c_bins, acc);

% Perform refined votings
rbins1 = load('rbins1.mat');
rbins2 = load('rbins2.mat');
rbins3 = load('rbins3.mat');
rbins4 = load('rbins4.mat');
rbins = [rbins1, rbins2, rbins3, rbins4];

r_acc = zeros(size(rbins1,1));
r_res = [0.27312, 0.09901, 0.02967, 0.01396]; % refined precision

for k = 0:3,
    bins = rbins(i,1+3*k:3*(k+1));
    bins = rotate_bins(bins);
    for i = 1:size(rbins1,1),
        for j = 1:size(G,1),
            if(abs(G(j,1)*bins(i,1) + G(j,2)*bins(i,2) + G(j,3)*bins(i,3) < r_res(k+1))),
                r_acc(i) = r_acc(i)+1;
            end
        end
    end
end