addpath('/media/hddData/Libraries/Scaramuzza_OCamCalib_v3.0_win/')
close all
load('cam3/calib_data_v0.mat')
ocam = calib_data.ocam_model;
theta0 = linspace(0.0, -pi+0.2,81);
M0 = [cos(theta0); zeros(size(theta0)); sin(theta0)];
m0 = world2cam(M0,ocam);

[xx, yy] = meshgrid(1:620, (1:4:4*36));
m = [xx(:)'; yy(:)'];
M = cam2world(m,ocam);

mc = [ocam.xc; ocam.yc];
Mc = cam2world(mc, ocam);

m1 = [round(m0(1,:)); round(m0(2,:)/4)*4];
M1 = cam2world(m1,ocam);
theta1 = acos(M1(1,:))-pi;
i = theta0 > 0;
theta1(i) = -theta1(i)-2*pi;
theta2 = asin(M1(3,:));

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

figure
plot(-theta0-pi)
hold all
plot(theta1)
plot(theta2)

mm = [m1(1,:)-1; m1(2,:)/4-1];
fid = fopen('calib_cam3.h','w');

fprintf(fid,'#define NB_SAMPLES %d\n\n', size(mm,2));

fprintf(fid,'cam_model px4_model= \n{{');
fprintf(fid,'%ff, ',ocam.ss);
fprintf(fid,'},\n\t%d,\n\t{',numel(ocam.ss)-1);
fprintf(fid,'%ff,',zeros(7,1));
fprintf(fid,'},\n\t%d,\n',7);
fprintf(fid,'\t%ff,\n\t%ff,\n\t',mc(1,:)-1,mc(2,:)/4-1);
fprintf(fid,'%ff,\n\t%ff,\n\t%ff,\n\t',ocam.c,ocam.d,ocam.e);
fprintf(fid,'%d,\n\t%d};\n\n',size(xx,2), size(xx,1));

fprintf(fid,'struct s_dir_2d {\n\tint16_t x[NB_SAMPLES];\n\tint16_t y[NB_SAMPLES];\n} s_dir_2d ={ 	{');
fprintf(fid,'%d,',mm(1,:));
fprintf(fid,'},\n\t {');
fprintf(fid,'%d,',mm(2,:));
fprintf(fid,'}};\n\n');

fprintf(fid,'struct s_dir_3d {\n\tfloat x[NB_SAMPLES];\n\tfloat y[NB_SAMPLES];\n\tfloat z[NB_SAMPLES];\n} s_dir_3d ={ 	{');
fprintf(fid,'%ff,',M1(1,:));
fprintf(fid,'},\n\t {');
fprintf(fid,'%ff,',M1(2,:));
fprintf(fid,'},\n\t {');
fprintf(fid,'%ff,',M1(3,:));
fprintf(fid,'}};\n\n');

fprintf(fid,'float s_dir_theta[NB_SAMPLES] = {');
fprintf(fid,'%ff,',theta1+pi/2);
fprintf(fid,'};\n');

fclose(fid);


