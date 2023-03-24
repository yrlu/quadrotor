function [G2L,L2G,yaw,pitch] = global2local(a,b)
%This function transfer the global pos vel acc into a corridor frame
%that its x-axis is along the line from a to b, its origin is at a,
%its y-axis is parrlel to the global y.
%---Input:
%a: the starting point of this corridor
%b: the end point of this corridor
%pos: position in global frame
%vel: velocity in global frame
%acc: acceleration in global frame
%---Output:
%G2L: the transformation matrix
%L2G: the reverse transformation matrix
%yaw: yaw angle in rad
%pitch: pitch angle in rad
%__________________________________________________________________________

vec = b - a;
%---Yaw rotation
yaw = atan2(vec(2),vec(1));
%---forward
T_yaw = yawMatrix(yaw);
%---reverse
T_yaw_r = yawMatrix(-yaw);

%---Pitch rotation
pitch = atan2(vec(3),norm(vec(1:2)));
%---forward
T_pitch = pitchMatrix(pitch);
%---reverse
T_pitch_r = pitchMatrix(-pitch);

%---Translation
T_trans = transMatrix(-a);
T_trans_r = transMatrix(a);

%---Outputs
G2L.free = T_pitch*T_yaw;
G2L.pos = T_pitch*T_yaw*T_trans;

L2G.free = T_yaw_r*T_pitch_r;
L2G.pos = T_trans_r*T_yaw_r*T_pitch_r;

