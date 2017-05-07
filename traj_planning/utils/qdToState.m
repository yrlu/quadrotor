function [x] = qdToState(qd)
% Converts state vector for simulation to qd struct used in hardware.
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

x = zeros(1,13); %initialize dimensions

x(1:3) = qd.pos;
x(4:6) = qd.vel;

Rot = RPYtoRot_ZXY(qd.euler(1), qd.euler(2), qd.euler(3));
quat = RotToQuat(Rot);

x(7:10) = quat;
x(11:13) = qd.omega;

end
