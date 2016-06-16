%--------------------------------------------------------------------------
%   PROCESS FLOW 81 DATA
%--------------------------------------------------------------------------
%% Process camera data
clear all;
load('flow81_cam.mat');
load('flow81_200fps.mat');

% Rescale and align ground time
offset = Time(1);
for i=1:length(Time),
    Time(i) = int32((Time(i) - offset)*1000);
end

% find NaN lines
nanLine = isnan(CamX) | isnan(GyroX);

% remove Nan lines
CamFrame_ = [];
CamTime_ = [];
CamX_ = [];
CamY_ = [];
CamZ_ = [];
GyroX_ = [];
GyroY_ = [];
GyroZ_ = [];

for i=1:length(nanLine),
    if nanLine(i) == 0;
        CamFrame_ = [CamFrame_ CamFrame(i)];
        CamTime_ = [CamTime_ CamTime(i)];
        CamX_ = [CamX_ CamX(i)];
        CamY_ = [CamY_ CamY(i)];
        CamZ_ = [CamZ_ CamZ(i)];
        GyroX_ = [GyroX_ GyroX(i)];
        GyroY_ = [GyroY_ GyroY(i)];
        GyroZ_ = [GyroZ_ GyroZ(i)];
    end
end

% remove delayed values (gyroscopes and images not synchronized)
delayedLine = CamTime_ > 10000;
CamFrame = [];
CamTime = [];
CamX = [];
CamY = [];
CamZ = [];
GyroX = [];
GyroY = [];
GyroZ = [];

for i = 1:length(delayedLine),
    if delayedLine(i) == 0;
        CamFrame = [CamFrame CamFrame_(i)];
        CamTime = [CamTime CamTime_(i)];
        CamX = [CamX CamX_(i)];
        CamY = [CamY CamY_(i)];
        CamZ = [CamZ CamZ_(i)];
        GyroX = [GyroX GyroX_(i)];
        GyroY = [GyroY GyroY_(i)];
        GyroZ = [GyroZ GyroZ_(i)];
    end
end

clear CamFrame_ CamTime_ CamX_ CamY_ CamZ_ GyroX_ GyroY_ GyroZ_ delayedLine nanLine i

% align CamFrame with 0
offset = CamFrame(1);
for i=1:length(CamFrame),
    CamFrame(i) = CamFrame(i) - offset;
end

% Rescale gyroscope measurements
for i=1:length(GyroX(1,:));
    GyroX(1,i) = GyroX(1,i)*200;
    GyroY(1,i) = GyroY(1,i)*200;
    GyroZ(1,i) = GyroZ(1,i)*200;
end

% Interpolate camera measurements
camtime = 0:5:CamFrame(length(CamFrame))+5; 
GyroX = interp1(CamFrame, GyroX, camtime);
GyroY = interp1(CamFrame, GyroY, camtime);
GyroZ = interp1(CamFrame, GyroZ, camtime);
CamX = interp1(CamFrame, CamX, camtime);
CamY = interp1(CamFrame, CamY, camtime);
CamZ = interp1(CamFrame, CamZ, camtime);

save('flow_81_cam_pro_ter.mat', 'CamX', 'CamY', 'CamZ', 'GyroX', 'GyroY', 'GyroZ');

%% Process motion capture data
% Estimate angular rates
quat = [QuatW, QuatX, QuatY, QuatZ];
timestep = 5*10^-3;

dquat = zeros(length(quat(:,1)), length(quat(1,:)));
dquat(1,:) = quatmultiply(quat(2,:), quatinv(quat(1,:))); % 2 points rule
for i = 2:length(quat(:,1))-1,
    dquat(i,:) = quatmultiply(quat(i+1,:), quatinv(quat(i-1,:))); % 3 points rule
end
dquat(length(quat(:,1)),:) = quatmultiply(quat(length(quat(:,1)),:), quatinv(quat(length(quat(:,1))-1,:))); % 2 points rule

quatAngles = quatmultiply(quatinv(quat), quatmultiply(dquat, quat));
angles = quatAngles(:,2:4);

rates = zeros(size(angles));
rates(1,:)  = 2*angles(1,:)/timestep;
for i = 2:length(angles(:,1))-1,
    rates(i,:) = 2*angles(i,:)/(2*timestep);
end
rates(length(angles(:,1)),:) = 2*angles(length(angles(:,1)),:)/timestep;

rateX = rates(:,1);
rateY = rates(:,2);
rateZ = rates(:,3);

% Estimate direction of translation
pos = [X, Y, Z];

dir = zeros(length(pos(:,1)), length(pos(1,:)));
dir(1,:) = (pos(2,:) - pos(1,:))/timestep; % 2 points rule
for i = 2:length(pos(:,1))-1,
    dir(i,:) = (pos(i,:) - pos(i-1,:))/(timestep); % 3 points rule
end
dir(length(pos(:,1)),:) = (pos(length(pos(:,1)),:) - pos(length(pos(:,1))-1,:))/timestep; % 2 points rule

qdir = [zeros(length(pos(:,1)),1), dir];
qdir = quatmultiply(quatmultiply(quatinv(quat),qdir),quat);

% rotM = quat2rotm(quatinv(quat));

% bodyDir = zeros(size(dir));
% for i = 1:length(angles(:,1)),
%     bodyDir(i,:) = (rotM(:,:,i)*dir(i,:)')';
% end
% bodyDir = normc(bodyDir')';
bodyDir = qdir(:,2:4);
velX = bodyDir(:,1)/1000;
velY = bodyDir(:,2)/1000;
velZ = bodyDir(:,3)/1000;
bodyDir = normr(bodyDir);

dirX = bodyDir(:,1);
dirY = bodyDir(:,2);
dirZ = bodyDir(:,3);

%clear angRates bodyDir dir dquat i invquat pos quat quatRates QuatX QuatY QuatW QuatZ rates rotM X Y Z

save('flow_81_200fps_pro_ter.mat', 'velX', 'velY', 'velZ', 'dirX', 'dirY', 'dirZ', 'rateX', 'rateY', 'rateZ', 'Time', 'ErrorMarkers');

%% Load full data
close all
clear all
load('flow_81_cam_pro_ter.mat');
load('flow_81_200fps_pro_ter.mat');

%% Synchronize manually
close all
% define delay
delay = 1445;

%% Plot angular rates measurements
% rates around X
figure;
hold on
h1 = plot(rateZ);
h2 = plot(GyroX(delay:length(GyroX)));
legend([h1, h2], 'Optitrack Z','Gyroscope X');
xlabel('Frames (~5 ms)');
ylabel('Angular rate around X');
ax = gca;
ax.XLim = [3000,6000];
ax.YLim = [-15, 15];
set(gca,'FontSize', 12);
hold off

% rates around Y
figure;
hold on
h1 = plot(rateX);
h2 = plot(GyroY(delay:length(GyroY)));
legend([h1, h2], 'Optitrack Z','Gyroscope Y');
xlabel('Frames (~5 ms)');
ylabel('Angular rate around Y');
ax = gca;
ax.XLim = [3000,6000];
ax.YLim = [-12, 12];
set(gca,'FontSize', 12);
hold off

% rates around Z
figure;
hold on
h1 = plot(rateY);
h2 = plot(GyroZ(delay:length(GyroZ)));
legend([h1, h2], 'Optitrack Y','Gyroscope Z');
xlabel('Frames (~5 ms)');
ylabel('Angular rate around Z');
ax = gca;
ax.XLim = [3000,6000];
ax.YLim = [-15, 15];
set(gca,'FontSize', 12);
hold off

%% Compare direction estimates
close all;

% crop data
CamX_ = CamX(delay:length(CamX))';
CamY_ = CamY(delay:length(CamY))';
CamZ_ = CamZ(delay:length(CamZ))';
GyroX_ = GyroX(delay:length(GyroX))';
GyroY_ = GyroY(delay:length(GyroY))';
GyroZ_ = GyroZ(delay:length(GyroZ))';
dirX_ = dirX(1:length(CamX_));
dirY_ = dirY(1:length(CamY_));
dirZ_ = dirZ(1:length(CamZ_));

%% Smooth motion capture measurements

% do not smooth
% dirX = dirX_;
% dirY = dirY_;
% dirZ = dirZ_;

% smooth
g = gausswin(200,5);
g = g/sum(g);

dirX = conv(dirX_, g, 'same');
dirY = conv(dirY_, g, 'same');
dirZ = conv(dirZ_, g, 'same');

% renormalize
dir = normr([dirX, dirY, dirZ]);
dirX = dir(:,1);
dirY = dir(:,2);
dirZ = dir(:,3);

%% Plot results from smoothing of motion capture measurements

% direction estimate along X
figure;
hold on
h1 = plot(dirZ_);
h2 = plot(dirZ);
legend([h1 h2], 'Raw', 'Smoothed');
ylabel('Direction estimate along X');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,12000];
set(gca,'FontSize', 12);
hold off;

% direction estimate along Y
figure;
hold on
h1 = plot(dirX_);
h2 = plot(dirX);
legend([h1 h2], 'Raw', 'Smoothed');
ylabel('Direction estimate along Y');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,12000];
set(gca,'FontSize', 12);
hold off;

% direction estimate along Z
figure;
hold on
h1 = plot(dirY_);
h2 = plot(dirY);
legend([h1 h2], 'Raw', 'Smoothed');
ylabel('Direction estimate along Z');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,12000];
set(gca,'FontSize', 12);
hold off;

%% Disambiguate camera estimates (to correspond to ground truth direction if possible)
for i=1:length(CamX_),
    if CamX_(i)*dirZ_(i)<0 && CamY_(i)*dirX_(i)<0 && CamZ_(i)*dirY_(i)<0,
        CamX_(i) = -CamX_(i);
        CamY_(i) = -CamY_(i);
        CamZ_(i) = -CamZ_(i);
    end
end
%% Smooth optic-flow measurements

% do not smooth
% CamX = CamX_;
% CamY = CamY_;
% CamZ = CamZ_;

% smooth
g = gausswin(200,5);
g = g/sum(g);

CamX = conv(CamX_, g, 'same');
CamY = conv(CamY_, g, 'same');
CamZ = conv(CamZ_, g, 'same');

% renormalize
cam = normr([CamX, CamY, CamZ]);
CamX = cam(:,1);
CamY = cam(:,2);
CamZ = cam(:,3);

%% Plot results from smoothing of optic-flow estimates
close all
% direction estimate along X
figure;
hold on
%h1 = plot(CamX_);
h2 = plot(CamX);
% legend([h1 h2], 'Raw', 'Smoothed');
ylabel('Direction estimate along X');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off;

% direction estimate along Y
figure;
hold on
% h1 = plot(CamY_);
h2 = plot(CamY);
% legend([h1 h2], 'Raw', 'Smoothed');
ylabel('Direction estimate along Y');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off;

% direction estimate along Z
figure;
hold on
% h1 = plot(CamZ_);
h2 = plot(CamZ);
% legend([h1 h2], 'Raw', 'Smoothed');
ylabel('Direction estimate along Y');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off;

%% Compare results along each axis
close all
% direction estimate along X 
figure;
hold on
h1 = plot(dirZ);
h2 = plot(CamX);
legend([h1 h2], 'Ground truth', 'Estimate');
ylabel('Direction estimate along X');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off;

% direction estimate along Y 
figure;
hold on
h1 = plot(dirX);
h2 = plot(CamY);
legend([h1 h2], 'Ground truth', 'Estimate');
ylabel('Direction estimate along Y');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off;

% direction estimate along Z
figure;
hold on
h1 = plot(dirY);
h2 = plot(CamZ);
legend([h1 h2], 'Ground truth', 'Estimate');
ylabel('Direction estimate along Z');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off;

%% Compute accuracy and statistics
% define variables 
dir_gt = [dirZ, dirX, dirY];
dir_est = [CamX, CamY, CamZ];

% compute error
accuracy_ = dot(dir_gt', dir_est');

% apply gaussian window
% g = gausswin(200,25);
% g = g/sum(g);
% accuracy_ = conv(accuracy_, g, 'same');

% compute statistics
acc_mean = mean(accuracy_(3000:6000));
acc_std  = std(accuracy_(3000:6000));
acc_max = max(accuracy_(3000:6000));
acc_min = min(accuracy_(3000:6000));

%% Plot accuracy and statistics
figure;
hold on
plot(accuracy_)
ylabel('Accuracy');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
hold off

% display results
fprintf('Mean accuracy: %f \n Std. dev.: %f \n Max: %f \n Min: %f \n', acc_mean, acc_std, acc_max, acc_min);

%% Test gyros vs accuracy

Gyro = [GyroX, GyroY, GyroZ];
Gyro_ = sqrt(sum(abs(Gyro).^2,2));
Gyro_ = Gyro_/max(Gyro_);

figure;
subplot(3,1,1);
hold on
h1 = plot(accuracy_);
h2 = plot(GyroX/max(GyroX));
ylabel('Accuracy');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
legend([h1,h2], 'Accuracy', 'GyroX');
hold off

subplot(3,1,2);
hold on
h1 = plot(accuracy_);
h2 = plot(GyroY/max(GyroY));
ylabel('Accuracy');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
legend([h1,h2], 'Accuracy', 'GyroY');
hold off

subplot(3,1,3);
hold on
h1 = plot(accuracy_);
h2 = plot(GyroZ/max(GyroZ));
ylabel('Accuracy');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
legend([h1,h2], 'Accuracy', 'GyroZ');
hold off

figure;
hold on
h1 = plot(accuracy_);
h2 = plot(Gyro_);
ylabel('Accuracy');
xlabel('Frames (~5 ms)');
ax = gca;
ax.XLim = [3000,6000];
set(gca,'FontSize', 12);
legend([h1,h2], 'Accuracy', 'Norm angular velocity (scaled)');
hold off

%% Process velocity
% g = gausswin(200,25);
% g = g/sum(g);
% plot(g);
% 
% velX_ = velX(1:length(CamZ));
% velY_ = velY(1:length(CamZ));
% velZ_ = velZ(1:length(CamZ));
% 
% velX = conv(velX_, g, 'same');
% velY = conv(velY_, g, 'same');
% velZ = conv(velZ_, g, 'same');
% 
% vel = [velX, velY, velZ];

%% Compute curvature of the trajectory
% timestep = 5*10^-3;
% 
% diffR = zeros(length(vel(:,1)), 3);
% diffR(1,:) = cross(vel(2,:), vel(1,:));
% for i = 2:length(diffR)-1,
%     diffR(i,:) = cross(vel(i,:),vel(i-1,:));
% end
% diffR(length(diffR),:) = cross(vel(length(diffR),:),vel(length(diffR)-1,:));
% 
% curv_ = asin(sqrt(sum(diffR.^2,2)))/timestep;
% 
%% Compute estimation error
% error_ = abs(CamZ') + abs(dirY);
% 
%% Compute best parameters for criteria
% % Parameters
% N = 500;
% lambda_ = 0.1:0.01:0.99; % Grid-search
% 
% best_cor = 0;
% best_lambda = 0;
% 
% for l = 1:length(lambda_),
%     lambda = lambda_(l);
%     
%     % compute criteria
%     
%     num = zeros(length(diffR),1);
% 
%     den = 0;
%     for n = 0:N,
%         den = den + lambda^n;
%     end
% 
%     for k = 1:length(diffR),
%         for n = 0:(min(k-1,N)),
%             num(k) = num(k) + curv_(k-n)*lambda^n;
%         end
%     end
%     crit_ = num/den;
%     
%     % compute correlation
%     corcoeffs = corrcoef(error_, crit_);
%     cor = abs(corcoeffs(2,1));
%     if cor > best_cor,
%         best_cor = cor;
%         best_lambda = lambda_(l);
%     end
% end
% 
%% Compute final criteria
% lambda = best_lambda;
% num = zeros(length(diffR),1);
% 
% den = 0;
% for n = 0:N,
%     den = den + lambda^n;
% end
% 
% for k = 1:length(diffR),
%     for n = 0:(min(k-1,N)),
%         num(k) = num(k) + curv_(k-n)*lambda^n;
%     end
% end
% crit_ = num/den;
% 
%% Plot error as a function of criteria
% [crit_s, order] = sort(crit_);
% figure;
% hold on
% plot(crit_s, error_);
