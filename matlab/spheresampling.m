clear all
close all
graphics_toolkit('gnuplot') % COMMENT THIS LINE IN MATLAB

%%% Latitude-Longitude method
%
%% parameters
%N = 30;
%r = 1;
%x = [];
%y = [];
%z = [];
%
%% main loop
%N_count = 0;
%a = 4*pi*r/N;
%d = sqrt(a);
%M_theta = round(pi/d);
%d_theta = pi/M_theta;
%d_phi = a/d_theta;
%for m=0:M_theta-1,
%  theta = pi*(m+0.5)/M_theta;
%  M_phi = round(2*pi*sin(theta)/d_phi);
%  for n = 0:M_phi-1,
%    phi = 2.0*pi*n/M_phi;
%    x = [x; r*sin(theta)*cos(phi)];
%    y = [y; r*sin(theta)*sin(phi)];
%    z = [z; cos(theta)];
%    N_count = N_count + 1;
%  end
%end
%
%% plots
%figure(1)
%hold on;
%axis('square');
%plot3(x,y,z,'b.');
%[x_, y_, z_] = sphere;
%mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
%hold off;
%display('The points are not equally spaced at the poles...');
%
%%% Fibonacci sphere method
%
%% parameters
%N = 60;
%r = 1;
%x = [];
%y = [];
%z = [];
%
%% main loop
%offset = 2/N;
%increment = pi*(3-sqrt(5));
%%for i=0:N-1,
%%  y_i = ((i*offset)-1) + (offset/2);
%%  r = sqrt(1 - y_i*y_i);
%%  phi = mod((i + 1),N)*increment;
%%  x_i = cos(phi)*r;
%%  z_i = sin(phi)*r;
%%  x = [x; x_i];
%%  y = [y; y_i];
%%  z = [z; z_i];
%%end
%phi = (1.0 + sqrt(5.0))/2.0;
%i = (-(N - 1):2:(N-1))';
%theta = 2.0*pi*i/phi;
%sphi = i/N;
%cphi = sqrt((N+i).*(N-i))/N;
%
%x = zeros(N,1);
%y = zeros(N,1);
%z = zeros(N,1);
%
%x = cphi.*sin(theta);
%y = cphi.*cos(theta);
%z = sphi;
%
%figure(2)
%hold on;
%axis('square');
%plot3(x,y,z,'b.');
%[x_, y_, z_] = sphere;
%mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
%hold off;
%display('The points are not equally spaced...');

%%------------------------------------------------------------------------------
%   Icosahedron method
%-------------------------------------------------------------------------------

% Computing refined bins
N_coarse = 42;
F_coarse = 2;

points = sphere_icos1_points(F_coarse, N_coarse);
x = points(1,:)';
y = points(2,:)';
z = points(3,:)';
theta_min = Inf;

for i=1:length(x),
  for j = i+1:length(y),
    theta_t = acos(dot([x(i), y(i), z(i)],[x(j), y(j), z(j)]));
    if(theta_t<theta_min),
      theta_min = theta_t;
    end  
  end
end

theta_min_deg = theta_min*180/pi;
printf('Precision in theta: %d \n', theta_min_deg);

figure(3)
hold on;
axis('square');
plot3(x,y,z,'b.');
[x_, y_, z_] = sphere;
mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
title('Coarse bins');
hold off;

% Save in text file
savefile = '/home/brice/coarse.txt'; % CHANGE FOLDER ON YOUR SYSTEM
myfile = fopen(savefile ,'wt');
fprintf(myfile,'X\tY\tZ\n');
coarse_bins = [x, y, z];
fprintf(myfile, '%g\t%g\t%g\n',coarse_bins');
fclose(myfile);

% results
% Precision of 15 degrees for F=4 (N=162) 
% Precision of 20 degrees for F=3 (N=92) = COARSE ?
% Precision of 31 degrees for F=2 (N=42) = COARSE ?

%-------------------------------------------------------------------------------

% Computing refined bins

% parameters 
N_refined = 1000;
F_refined = 14;

% compute refined precision
points = sphere_icos1_points(F_refined, N_refined);
x = points(1,:)';
y = points(2,:)';
z = points(3,:)';
theta_min = Inf;

for i=1:length(x),
  for j = i+1:length(y),
    theta_t = acos(dot([x(i), y(i), z(i)],[x(j), y(j), z(j)]));
    if(theta_t<theta_min),
      theta_min = theta_t;
    end  
  end
end
theta_min_deg = theta_min*180/pi;
printf('Precision in theta: %d \n', theta_min_deg);

% compute refined bins
ref_bins = [];
for i=1:length(x),
   theta_t = acos(dot([0, 0, 1],[x(i), y(i), z(i)]));
   if(theta_t<31*pi/360),
    ref_bins = [ref_bins; [x(i), y(i), z(i)]];
   end  
end

% Plot refined bins
figure(4)
hold on;
axis('square');
plot3(ref_bins(:,1),ref_bins(:,2),ref_bins(:,3),'b.');
[x_, y_, z_] = sphere;
mesh(x_, y_, z_, 'Facecolor', 'b', 'Facealpha', 0.1, 'Edgecolor', 'none');
title('Refined bins');
hold off;

% Refined precision of <4 degrees for F=14 (N_ref=57 for 20 degrees in coarse)
% Refined precision of >3 degrees for F=16 (N_ref=77 for 20 degrees in coarse)
% Refined precision of <4 degrees for F=14 (N_ref=139 for 31 degrees in coarse)
% Refined precision of <4 degrees for F=14 (N_ref=37 for 31/2 degrees in coarse)

% Save in text file
savefile = '/home/brice/refined.txt'; % CHANGE FOLDER ON YOUR SYSTEM
myfile = fopen(savefile ,'wt');
fprintf(myfile,'X\tY\tZ\n');
fprintf(myfile, '%g\t%g\t%g\n',ref_bins');
fclose(myfile);
