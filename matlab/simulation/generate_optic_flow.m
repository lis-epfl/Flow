function [ of ] = generate_optic_flow(v, w, d, D)
% GENERATE_OPTIC_FLOW Generates optic-flow vectors on the unit sphere given
% translational and angular velocity vectors and sampling directions
%   v:  translational velocity (in body frame)
%   w:  angular velocity (in body frame)
%   d:  sampling directions (in body frame)
%   D:  depth of the scene

% Define variables
of = zeros(size(d));

% Normalize d
d = normr(d);
V = repmat(v,size(d,1),1);
W = repmat(w,size(d,1),1);

% Compute optic-flow on the unit sphere (first order approximation)
ofr = -cross(W,d);
oft = -(V-(repmat(dot(V,d,2),1,3).*d))/D;
of = ofr + oft;

end

