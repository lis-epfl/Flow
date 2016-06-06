function [ R ] = aa2mat(u_x, u_y, u_z, c, s, t )
% AA2MAT Computes coefficients of the rotation matrix
%   [u_x, u_y, u_z]:    cartesian coordinates of the axis
%   c:  cosine of the angle
%   s:  sine of the angle
%   t: 1-c
    R(1) = u_x^2*t + c;
	R(2) = u_x*u_y*t - u_z*s;
	R(3) = u_x*u_z*t + u_y*s;
	R(4) = u_x*u_y*t + u_z*s;
	R(5) = u_y^2*t + c;
	R(6) = u_y*u_z*t - u_x*s;
	R(7) = u_x*u_z*t - u_y*s;
	R(8) = u_y*u_z*t + u_x*s;
	R(9) = u_z^2*t + c;
end

