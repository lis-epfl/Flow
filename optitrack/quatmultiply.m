function n  = quatmultiply( q, r )
%QUATMULTIPLY Computes the product of two quaternions
m = length(q(:,1));
n = zeros(m,4);
for i = 1:m,
    n(i,1) = (r(i,1)*q(i,1) - r(i,2)*q(i,2) - r(i,3)*q(i,3) - r(i,4)*q(i,4));
    n(i,2) = (r(i,1)*q(i,2) + r(i,2)*q(i,1) - r(i,3)*q(i,4) + r(i,4)*q(i,3));
    n(i,3) = (r(i,1)*q(i,3) + r(i,2)*q(i,4) + r(i,3)*q(i,1) - r(i,4)*q(i,2));
    n(i,4) = (r(i,1)*q(i,4) - r(i,2)*q(i,3) + r(i,3)*q(i,2) + r(i,4)*q(i,1));
    %fprintf('n = (%f, %f, %f, %f)\n', n(i,1), n(i,2), n(i,3), n(i,4));
end

end

