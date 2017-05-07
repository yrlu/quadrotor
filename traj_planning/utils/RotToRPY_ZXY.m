function [phi,theta,psi] = RotToRPY_ZXY(R)
%RotToRPY_ZXY Extract Roll, Pitch, Yaw from a world-to-body Rotation Matrix
%   The rotation matrix in this function is world to body [bRw] you will
%   need to transpose the matrix if you have a body to world [wRb] such
%   that [wP] = [wRb] * [bP], where [bP] is a point in the body frame and
%   [wP] is a point in the world frame
%   written by Daniel Mellinger
%   bRw = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),
%           cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),
%          -cos(phi)*sin(theta)]
%         [-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi)]
%         [ cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
%           sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
%           cos(phi)*cos(theta)]

phi = asin(R(2,3));
psi = atan2(-R(2,1)/cos(phi),R(2,2)/cos(phi));
theta = atan2(-R(1,3)/cos(phi),R(3,3)/cos(phi));

end
