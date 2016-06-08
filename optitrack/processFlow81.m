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

save('flow_81_cam_pro.mat', 'CamX', 'CamY', 'CamZ', 'GyroX', 'GyroY', 'GyroZ');

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
bodyDir = normr(bodyDir);

dirX = bodyDir(:,1);
dirY = bodyDir(:,2);
dirZ = bodyDir(:,3);

%clear angRates bodyDir dir dquat i invquat pos quat quatRates QuatX QuatY QuatW QuatZ rates rotM X Y Z

save('flow_81_200fps_pro.mat', 'dirX', 'dirY', 'dirZ', 'rateX', 'rateY', 'rateZ', 'Time', 'ErrorMarkers');

%% Load full data
close all
clear all
load('flow_81_cam_pro.mat');
load('flow_81_200fps_pro.mat');

%% Synchronize measurements (convolution)

% Correlation with motion capture data
% imax = length(rateZ(:,1));
% jmax = length(GyroZ(:,1));
% conv = zeros(2*jmax+1, 1);
% for j = -jmax:jmax,
%     for i = 1:imax,
%         if (i-j>0 && i-j<length(rateZ(:,1))),
%             conv(i+jmax+1) = conv(j+jmax+1) + GyroZ(i-j)*rateY(i);
%         end
%     end
% end
% delay = find(conv==max(conv)) - jmax - 1;

%% Synchronize manually
% define delay
delay = 1445;

% plot results
% rates around X
figure;
hold on
h1 = plot(rateZ);
h2 = plot(GyroX(delay:length(GyroX)));
legend([h1, h2], 'Optitrack Z','Gyroscope X');
hold off

% rates around Y
figure;
hold on
h1 = plot(rateX);
h2 = plot(GyroY(delay:length(GyroY)));
legend([h1, h2], 'Optitrack Z','Gyroscope Y');
hold off

% rates around Z
figure;
hold on
plot(rateY);
plot(GyroZ(delay:length(GyroZ)));
legend([h1, h2], 'Optitrack Y','Gyroscope Z');
hold off

%% Compare direction estimates
% crop data
CamZ_ = CamZ(delay:length(CamZ));
dirY_ = dirY(1:length(CamZ_));

%% Smooth motion capture measurements
g = gausswin(200,5);
g = g/sum(g);
plot(g);

% dirX_ = dirX;
% dirY_ = dirY;
% dirZ_ = dirZ;

% dirX = conv(dirX_, g, 'same');
dirY = conv(dirY_, g, 'same');
% dirZ = conv(dirZ_, g, 'same');

% Plot result
figure;
hold on
h1 = plot(dirY_);
h2 = plot(dirY);
legend([h1 h2], 'smoothed', 'raw');
hold off;

%% Smooth optic-flow measurements
g = gausswin(200,5);
g = g/sum(g);
plot(g);

% CamX_ = CamX;
% CamY_ = CamY;
% CamZ_ = CamZ;

% CamX = conv(CamX_, g, 'same');
% CamY = conv(CamY_, g, 'same');
CamZ = conv(CamZ_, g, 'same');

% Plot result
figure;
hold on
h1 = plot(CamZ_);
h2 = plot(CamZ);
legend([h1 h2], 'raw', 'smoothed');
hold off;

%% Plot raw measurements
figure;
hold on
h2 = plot(abs(CamZ));
drawnow;
pause;
h1 = plot(abs(dirY));
legend([h1, h2], 'Optitrack Y','Camera Z');
hold off

% % plot error
% error = CamZ_'+abs(dirY_);
% figure;
% hold on
% plot(error);
