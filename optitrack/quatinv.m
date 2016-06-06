function [invq] = quatinv(q)
%QUATINV Computes the inverse of the quaternion
n = length(q(:,1));
m = length(q(1,:));
invq = zeros(n,m);
for i = 1:n,
    %invnormq = 1/(q(i,1)^2 + q(i,2)^2 + q(i,3)^2 + q(i,4)^2);
    invq(i,:) = [q(i,1), -q(i,2), -q(i,3), -q(i,4)];
end


end

