addpath('/media/hddData/Libraries/Scaramuzza_OCamCalib_v3.0_win/')
close all
load('cam3/calib_data_v0.mat')
ocam = calib_data.ocam_model;


ocam.xc = ocam.xc -1;   % adjust optical center to C convention (0 index)
ocam.yc = ocam.yc -1;   % adjust optical center to C convention (0 index)
ocam.ss = -ocam.ss;

theta0 = linspace(0.2, pi,81);
M0 = [-cos(theta0); zeros(size(theta0)); sin(theta0)];
m0 = world2cam(M0,ocam);

[xx, yy] = meshgrid(0:619, (0:4:((4*36)-1)));
m = [xx(:)'; yy(:)'];
M = cam2world(m,ocam);

mc = [ocam.xc; ocam.yc];
Mc = cam2world(mc, ocam);

m1 = [round(m0(1,:)); round(m0(2,:)/4)*4];
M1 = cam2world(m1,ocam);
theta1 = atan2(M1(1,:), M1(3,:))
%i = theta0 > 0;
%theta1(i) = -theta1(i)-2*pi;
%theta2 = asin(M1(3,:));

scatter(m(1,:),m(2,:))
hold on
scatter(m0(1,:),m0(2,:))
scatter(mc(1,:),mc(2,:))

figure
scatter3(M(1,:),M(2,:),M(3,:))
hold on
scatter3(M0(1,:),M0(2,:),M0(3,:))
scatter3(Mc(1,:),Mc(2,:),Mc(3,:))
scatter3(M1(1,:),M1(2,:),M1(3,:))
xlabel('x')
ylabel('y')
zlabel('z')

figure
plot(theta1,'b')
hold all
plot(theta0,'r')
plot(theta2,'g')
title('theta')
% ocam.ss = -ocam.ss;
%ocam.d = -ocam.d;
%ocam.e = -ocam.e;

mm = [m1(1,:); m1(2,:)/4];
fid = fopen('calib_cam3.h','w');
format long e
fprintf(fid,'#define NB_SAMPLES %d\n\n', size(mm,2));

fprintf(fid,'cam_model px4_model= \n{{');
fprintf(fid,'%.20ff, ',ocam.ss);
fprintf(fid,'},\n\t%d,\n\t{',numel(ocam.ss));
fprintf(fid,'%ff,',zeros(7,1));
fprintf(fid,'},\n\t%d,\n',7);
fprintf(fid,'\t%ff,\n\t%ff,\n\t',mc(1,:),mc(2,:));
fprintf(fid,'%.20ff,\n\t%.20ff,\n\t%.20ff,\n\t',ocam.c,ocam.d,ocam.e);
fprintf(fid,'%d,\n\t%d};\n\n',size(xx,2), size(xx,1));

fprintf(fid,'struct s_dir_2d {\n\tint16_t x[NB_SAMPLES];\n\tint16_t y[NB_SAMPLES];\n} s_dir_2d ={ 	{');
fprintf(fid,'%d,',m1(1,:));
fprintf(fid,'},\n\t {');
fprintf(fid,'%d,',m1(2,:)/4);
fprintf(fid,'}};\n\n');

fprintf(fid,'struct s_dir_3d {\n\tfloat x[NB_SAMPLES];\n\tfloat y[NB_SAMPLES];\n\tfloat z[NB_SAMPLES];\n} s_dir_3d ={ 	{');
fprintf(fid,'%ff,',M1(1,:));
fprintf(fid,'},\n\t {');
fprintf(fid,'%ff,',M1(2,:));
fprintf(fid,'},\n\t {');
fprintf(fid,'%ff,',M1(3,:));
fprintf(fid,'}};\n\n');

fprintf(fid,'float s_dir_theta[NB_SAMPLES] = {');
fprintf(fid,'%ff,',theta1);
fprintf(fid,'};\n');

fclose(fid);

for i = 1:size(m1,2)

u = m1(1,i);
v = m1(2,i);
invdet  = 1/(ocam.c-ocam.d*ocam.e);

xp = invdet*(    (u - ocam.xc) - ocam.d*(v - ocam.yc) );
yp = invdet*( -ocam.e*(u - ocam.xc) + ocam.c*(v - ocam.yc) );

r   = sqrt(xp.^2 + yp.^2);
zp  = ocam.ss(1);

r_i = 1;

for j = 2:numel(ocam.ss)
	   r_i = r_i*r;
	   zp  = zp + r_i*ocam.ss(j);
end

 n = sqrt(xp.^2 + yp.^2 + zp.^2);
 xp = xp/n;
 yp = yp/n;
 zp = zp/n;

MP(:,i) = [xp;yp;zp];

end


figure
scatter3(M1(1,:),M1(2,:),M1(3,:),'b');
hold on
scatter3(MP(1,:),MP(2,:),MP(3,:),'r');
