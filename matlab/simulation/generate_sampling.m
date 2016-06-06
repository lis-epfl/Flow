function [ p ] = generate_sampling( size, N )
%GENERATE_SAMPLING Generates sampling pixels given a field of view and
% the number of points
%   size:   size of the camera image [w,h]
%   N:      number of points (might not be exactly the size of d)

%%% Evenly distributed points on camera image

width = size(1);
height = size(2);
nPoints = N;

totalArea = width*height;
pointArea = totalArea/nPoints;
w = width/sqrt(nPoints);
h = height/sqrt(nPoints);

u = [];
v = [];

for i = w/2:w:width,
  for j = h/2:h:height,
    v = [v; floor(i)];
    u = [u; floor(j)];
  end
end

p = [u,v];
end

