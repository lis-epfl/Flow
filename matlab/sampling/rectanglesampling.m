clear all
close all
graphics_toolkit('gnuplot') % COMMENT THIS LINE IN MATLAB

%%% Evenly distributed points on camera image
%
%width = 160;
%height = 120;
%nPoints = 125;
%
%totalArea = width*height;
%pointArea = totalArea/nPoints;
%w = width/sqrt(nPoints);
%h = height/sqrt(nPoints);
%
%figure;
%hold on;
%
%x = [];
%y = [];
%
%for(i = w/2:w:width),
%  for(j = h/2:h:height),
%    y = [y; floor(i)];
%    x = [x; floor(j)];
%    plot(x, y, 'bo');
%  end
%end
%hold off;
%
%rectangle_sampling = [x, y];
%dlmwrite('rectangle_sampling.csv', rectangle_sampling, 'delimiter', ',', 'precision', 3);
%
%%% Evenly distributed points on spherical projection
%
%% Computing refined bins
%
%fov = 100;
%
%% parameters 
%N_refined = 55;
%F_refined = 8;
%
%% compute refined bins
%points = sphere_icos1_points(F_refined, N_refined);
%x = [];
%y = [];
%z = [];
%
%for i=1:length(points(1,:)),
%   theta_t = acos(dot([0, 0, -1],[points(1,i), points(2,i), points(3,i)]));
%   if(theta_t<=fov*pi/360),
%    x = [x; points(1,i)'];
%    y = [y; points(2,i)'];
%    z = [z; points(3,i)'];
%   end  
%end
%
%% plot refined bins
%figure(4)
%hold on;
%axis('square');
%plot3(x, y, z, 'b.');
%[x_, y_, z_] = sphere;
%mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
%title('Refined bins');
%hold off;
%
%% reproject on camera image
%u = zeros(length(x), 1);
%v = zeros(length(y), 1);
%u_ = zeros(length(x), 1);
%v_ = zeros(length(y), 1);
%norm_ = zeros(length(x), 1);
%invnorm_ = zeros(length(x), 1);
%theta = zeros(length(x),1);
%rho = zeros(length(x),1);
%dist = zeros(length(x),1);
%
%invpol         = [-98.889649, -60.099030, -3.523247, -11.584154, -10.704617, -4.911849, -0.899849]; 
%xc             = 56.232012;
%yc             = 77.63939272; 
%c              = 1.001183;
%d              = 0.001337;
%e              = 0.002268;
%length_invpol  = 7;
%
%for i=1:length(x),
%  norm_(i)           = sqrt(x(i)*x(i) + y(i)*y(i));
%
%  if (norm_(i) != 0)
%      theta(i)          = atan2(z(i),norm_(i));
%      invnorm_(i) = 1/norm_(i);
%      t  = theta(i);
%      rho(i) = invpol(1);
%      t_i = 1;
%    
%      for k = 2:length_invpol,
%         t_i = t;
%         rho(i) += t_i*invpol(k);
%      endfor
%      
%      dist(i) = invnorm_(i)*rho(i);
%      u_(i) = x(i)*dist(i);
%      v_(i) = y(i)*dist(i);
%      
%      u(i) = u_(i)*c + v_(i)*d + xc;
%      v(i) = u_(i)*e + v_(i)   + yc;
%  else
%    u(i) = xc;
%    v(i) = yc;
%  endif
%endfor
%
%% plot reprojections
%figure(5)
%axis('square');
%plot(u, v, 'b.');
%title('Sampling pixels');
% 
%spherical_sampling = [floor(u), floor(v)];
%dlmwrite('spherical_sampling.csv', spherical_sampling, 'delimiter', ',', 'precision', 3);

%% Evenly distributed points points on spherical projection

% Computing refined bins

% parameters 
N_refined = 55;
F_refined = 6;

% compute refined bins
points = sphere_icos1_points(F_refined, N_refined);
x = [];
y = [];
z = [];

for i=1:length(points(1,:)),
   if(points(3,i)<=0),
    x = [x; points(1,i)'];
    y = [y; points(2,i)'];
    z = [z; points(3,i)'];
   end  
end

% plot refined bins
figure(4)
hold on;
axis('square');
plot3(x, y, z, 'b.');
[x_, y_, z_] = sphere;
mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
title('Refined bins');
hold off;

% reproject on camera image
u = zeros(length(x), 1);
v = zeros(length(y), 1);
u_ = zeros(length(x), 1);
v_ = zeros(length(y), 1);
norm_ = zeros(length(x), 1);
invnorm_ = zeros(length(x), 1);
theta = zeros(length(x),1);
rho = zeros(length(x),1);
dist = zeros(length(x),1);

invpol         = [-98.889649, -60.099030, -3.523247, -11.584154, -10.704617, -4.911849, -0.899849]; 
xc             = 56.232012;
yc             = 77.63939272; 
c              = 1.001183;
d              = 0.001337;
e              = 0.002268;
length_invpol  = 7;

for i=1:length(x),
  norm_(i)           = sqrt(x(i)*x(i) + y(i)*y(i));

  if (norm_(i) != 0)
      theta(i)          = atan2(z(i),norm_(i));
      invnorm_(i) = 1/norm_(i);
      t  = theta(i);
      rho(i) = invpol(1);
      t_i = 1;
    
      for k = 2:length_invpol,
         t_i = t;
         rho(i) += t_i*invpol(k);
      endfor
      
      dist(i) = invnorm_(i)*rho(i);
      u_(i) = x(i)*dist(i);
      v_(i) = y(i)*dist(i);
      
      u(i) = u_(i)*c + v_(i)*d + xc;
      v(i) = u_(i)*e + v_(i)   + yc;
  else
    u(i) = xc;
    v(i) = yc;
  endif
endfor

% select points on camera image
cam_u = [];
cam_v = [];
for i = 1:length(x),
  if u(i) <= 110 && u(i) >= 10 && v(i) <= 150 && v(i) >= 10,
    cam_u = [cam_u; u(i)];
    cam_v = [cam_v; v(i)];
  end
end

% plot reprojections
figure(5)
axis('square');
plot(cam_u, cam_v, 'b.');
title('Sampling pixels');
 
spherical_sampling_2 = [int32(cam_u), int32(cam_v)];
dlmwrite('spherical_sampling_2.csv', spherical_sampling_2, 'delimiter', ',', 'precision', 3);
