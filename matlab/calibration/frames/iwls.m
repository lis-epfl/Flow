function [x,xstd] = iwls(h, z, weight)
% iterative weighted least square that takes 3 rotations (h) and a 1D
% optic-flow value (z) as input, and matches the three coefficients (x) that
% correspond to a row of the rotation matrix describing the optic-flow
% sensor's orientation... yek
N = length(z);

x = NaN(3,N);
P = NaN(3,3,N);
xstd = NaN(3,N);

x(:,1) = [0 0 0];
P(:,:,1) = eye(3);
xstd(:,1) = sqrt(diag(P(:,:,1)));

for i = 2:N
    
    if weight(i) > 0
        H = h(i,:);
        R = (1/weight(i))^2;
        P = P(:,:,i-1);
        
        K = P*H'*inv(H*P*H' + R);
        x(:,i) = x(:,i-1) + K*(z(i) - H*x(:,i-1));
        P(:,:,i) = (eye(3) - K*H)*P;
    else
        x(:,i)   = x(:,i-1);
        P(:,:,i) = P(:,:,i-1);
    end
    xstd(:,i) = sqrt(diag(P(:,:,i)));
end
