function [ P ] = back_project( p, model )
%BACK_PROJECTION Back-project pixels on the unit sphere given the camera model 
%   model: camera model
%   p: samplign pixels
%

% Define local variables
pol         = model.pol;
xc          = model.xc;
yc          = model.yc; 
c           = model.c;
d           = model.d;
e           = model.e;
length_pol  = model.length_pol;

invdet  = 1/(c-d*e);

u = p(:,1);
v = p(:,2);

% back-projection of u and v 
xp = invdet*(    (u - xc) - d*(v - yc) );
yp = invdet*( -e*(u - xc) + c*(v - yc) );

r   = sqrt(xp.^2 + yp.^2); % distance [pixels] of  the point from the image center

zp  = pol(1);
r_i = 1;

for i = 2:length_pol,
    r_i = r_i.*r;
    zp  = zp + r_i*pol(i);
end

P = [xp, yp, zp];
end

