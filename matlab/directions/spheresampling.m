clear all
close all
graphics_toolkit('gnuplot') % COMMENT THIS LINE IN MATLAB

%%------------------------------------------------------------------------------
%   Icosahedron method
%-------------------------------------------------------------------------------

% computing coarse bins
N_coarse = 25;
F_coarse = 2;

% initialize coordinates
x = [];
y = [];
z = [];

% compute coarse bins
points = sphere_icos1_points(F_coarse, N_coarse);
for(i=1:length(points(1,:))),
  if(points(3,i)<=0),
    x = [x; points(1,i)'];
    y = [y; points(2,i)'];
    z = [z; points(3,i)'];
  end
end

% compute precision
theta_min = Inf;
for i=1:length(x),
  for j = i+1:length(y),
    theta_t = acos(dot([x(i), y(i), z(i)],[x(j), y(j), z(j)]));
    if(theta_t<theta_min),
      theta_min = theta_t;
    end  
  end
end

coarse_prec = theta_min*180/pi;
printf('Coarse precision in theta: %d \n', coarse_prec);

% plot coarse bins
figure(3)
hold on;
axis('square');
plot3(x,y,z,'b.');
[x_, y_, z_] = sphere;
mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
title('Coarse bins');
hold off;

% save in text file
coarse_bins = [x, y, z];
dlmwrite('coarse_bins.csv', coarse_bins, 'delimiter', ',', 'precision', 6);

% results
% Precision of 15 degrees for F=4 (N=162 or N_hemi=89) 
% Precision of 20 degrees for F=3 (N=92 or N_hemi=52)
% Precision of 31 degrees for F=2 (N=42 or N_hemi=25)

%-------------------------------------------------------------------------------

% Computing refined bins

% parameters 
N_refined = 42;
F_refined = 15;

% compute refined bins
points = sphere_icos1_points(F_refined, N_refined);
x = [];
y = [];
z = [];

for i=1:length(points(1,:)),
   theta_t = acos(dot([0, 0, -1],[points(1,i), points(2,i), points(3,i)]));
   if(theta_t<=coarse_prec*pi/360),
    x = [x; points(1,i)'];
    y = [y; points(2,i)'];
    z = [z; points(3,i)'];
   end  
end

% compute refined precision
theta_min = Inf;
for i=1:length(x),
  for j = i+1:length(y),
    theta_t = acos(dot([x(i), y(i), z(i)],[x(j), y(j), z(j)]));
    if(theta_t<theta_min),
      theta_min = theta_t;
    end  
  end
end
refined_prec = theta_min*180/pi;
printf('Refined precision in theta: %d \n', refined_prec);

% plot refined bins
figure(4)
hold on;
axis('square');
plot3(x, y, z, 'b.');
[x_, y_, z_] = sphere;
mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
title('Refined bins');
hold off;

% Save in csv file
refined_bins = [x, y, z];
dlmwrite('refined_bins.csv', refined_bins,'delimiter', ',', 'precision', 6);

% Refined precision of ~4.6 degrees for F=14 (N_ref=37 for 31 degrees in coarse)
% Refined precision of ~4.4 degrees for F=15 (N_ref=42 for 31 degrees in coarse)
% Refined precision of ~4.4 degrees for F=16 (N_ref=43 for 31 degrees in coarse)
% Refined precision of ~3.8 degrees for F=17 (N_ref=48 for 31 degrees in coarse)
% Refined precision of ~3.5 degrees for F=18 (N_ref=57 for 31 degrees in coarse)
% Refined precision of ~3.4 degrees for F=19 (N_ref=64 for 31 degrees in coarse)
% Refined precision of ~3.2 degrees for F=20 (N_ref=72 for 31 degrees in coarse)