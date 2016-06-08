function [ bins ] = rotate_bins( rbins, d_est )
%ROTATE_BINS Returns the rotated voting bins
%   rbins: voting bins to be rotated
%   d_est: previous estimate (and main axis of the rotated bins)

bins = zeros(length(rbins));
best_x = d_est(1);
best_y = d_est(2);
best_z = d_est(3);

if(best_x == 0.0 && best_y == 0.0 && best_z == -1.0),
    for i=1:size(bins,1),			
        bins(i,1) = rbins(i,1);
        bins(i,2) = rbins(i,1);
        bins(i,3) = rbins(i,1);
    end
else
    % axis (gives the actual rotation direction)
    u_x = 1.0*best_y; 	% 0.0 * best_z - (-1.0) * best_y
    u_y = -1.0*best_x; 	% -1.0 * best_x - 0.0 * best_z
    u_z = 0.0;          % 0.0 * best_y - 0.0 * best_x
    u = [u_x u_y u_z];
    
    % angle
    c = -best_z;    % cosine
    s = sqrt(1 - c^2);    % sine
    t = 1.0 - c;
    
    % normalize axis
    u = normr(u);

    % compute rotation matrix (saves time)
    R = aa2mat(u(1), u(2), u(3), c, s, t);
%     r = vrrotvec([0 0 -1], d_est);
%     R = vrrotvec2mat(r);
%     R = R(:)';

    % compute rotated refined bins
    for i=1:length(bins),
        bins(i,1) = rbins(i,1)*R(1) + rbins(i,2)*R(2) + rbins(i,3)*R(3);
        bins(i,2) = rbins(i,1)*R(4) + rbins(i,2)*R(5) + rbins(i,3)*R(6);
        bins(i,3) = rbins(i,1)*R(7) + rbins(i,2)*R(8) + rbins(i,3)*R(9);
    end
end

end

