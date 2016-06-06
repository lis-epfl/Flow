function [ n ] = get_normal_vector(f, d)
% GET_NORMAL_VECTOR Computes normal vectors corresponding to great circles
% defined by input viewing directions and optic-flow
%   f:  optic-flow
%   d:  viewing directions
%

% Compute normal vectors
n = cross(d,f);

% Normalize normal vectors
n = normr(n);

end

