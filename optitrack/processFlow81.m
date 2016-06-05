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
GyroX = interp1(CamFrame, GyroX, Time);
GyroY = interp1(CamFrame, GyroY, Time);
GyroZ = interp1(CamFrame, GyroZ, Time);
CamX = interp1(CamFrame, CamX, Time);
CamY = interp1(CamFrame, CamY, Time);
CamZ = interp1(CamFrame, CamZ, Time);

save('flow_81_cam_pro.mat', 'CamX', 'CamY', 'CamZ', 'GyroX', 'GyroY', 'GyroZ');

%% Process motion capture data
% Estimate angular rates
quat = [QuatW, QuatX, QuatY, QuatZ]; 

dquat = zeros(length(quat(:,1)), length(quat(1,:)));
dquat(1,:) = (quat(2,:) - quat(1,:))/(5*10^-3); % 2 points rule
for i = 2:length(quat(:,1))-1,
    dquat(i,:) = (quat(i+1,:) - quat(i-1,:))/(10^-2); % 3 points rule
end
dquat(length(quat(:,1)),:) = (quat(length(quat(:,1)),:) - quat(length(quat(:,1))-1,:))/(5*10^-3); % 2 points rule

invquat = quatinv(quat);

quatRates = 2*quatmultiply(dquat, invquat);
angRates = quatRates(:,2:4);

rotM = quat2rotm(invquat);
rates = zeros(size(angRates));
for i = 1:length(angRates(:,1)),
    rates(i,:) = (rotM(:,:,i)*angRates(i,:)')';
end
rateX = rates(:,1);
rateY = rates(:,2);
rateZ = rates(:,3);

% Estimate direction of translation
pos = [X, Y, Z];

dir = zeros(length(pos(:,1)), length(pos(1,:)));
dir(1,:) = (pos(2,:) - pos(1,:))/(5*10^-3); % 2 points rule
for i = 2:length(pos(:,1))-1,
    dir(i,:) = (pos(i+1,:) - pos(i-1,:))/(10^-2); % 3 points rule
end
dir(length(pos(:,1)),:) = (pos(length(pos(:,1)),:) - pos(length(pos(:,1))-1,:))/(5*10^-3); % 2 points rule

bodyDir = zeros(size(dir));
for i = 1:length(angRates(:,1)),
    bodyDir(i,:) = (rotM(:,:,i)*dir(i,:)')';
end
bodyDir = normc(bodyDir')';

dirX = bodyDir(:,1);
dirY = bodyDir(:,2);
dirZ = bodyDir(:,3);

%clear angRates bodyDir dir dquat i invquat pos quat quatRates QuatX QuatY QuatW QuatZ rates rotM X Y Z

save('flow_81_200fps_pro.mat', 'dirX', 'dirY', 'dirZ', 'rateX', 'rateY', 'rateZ', 'Time', 'ErrorMarkers');

%% Load full data
clear all
load('flow_81_cam_pro.mat');
load('flow_81_200fps_pro.mat');

%% Synchronize measurements (convolution)

% Correlation with motion capture data
imax = floor(length(rateZ(:,1)));
jmax = length(GyroZ(:,1));
conv = zeros(2*imax+1, 1);
for i = -imax:imax,
    for j = jmax,
        if (j-i>0 && j-i<length(rateZ(:,1))),
            conv(i+imax+1) = conv(i+imax+1) + GyroZ(j)*rateZ(j-i);
        end
    end
end
find(conv==max(conv))