function [ qf ] = quatrot( qi, qr )
%QUATROT Rotates a quaternion
qinv = quatinv(qr);
qf = quatmultiply(qinv, qi);
qf = quatmultiply(qf, qr);

end

