function R = rotFromQuat(quat)
% compute rotation matrix from input quaternion 

qx = quat(1);
qy = quat(2);
qz = quat(3); 
qw = quat(4);

R = [ 2*(qw*qw + qx*qx)-1,   2*(qx*qy - qw*qz)  ,  2*(qx*qz + qw*qy)   ; ...
      2*(qx*qy + qw*qz),     2*(qw*qw + qy*qy)-1,  2*(qy*qz - qw*qx)   ; ... 
      2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx)  ,  2*(qw*qw + qz*qz)-1];
