function [vM_c,aM_c,jM_c]=tiltConstraintsApprx(vM,aM,jM,pitch)
%This function tilt the global cylinder shape constraints into the
%corridor frame while approximately maximize the resulted cublid's volume.
%Note this is only an approximated maximization.
%---Input:
%vM: global maximum velocity [horizontal, vertical]';
%aM: global maximum acceleration [horizontal, vertical]';
%jM: global maximum jerk [horizontal, vertical]';
%---Output:
%vM_c: corridor frame max velocity [x,y,z]';
%aM_c: corridor frame max acceleration [x,y,z]';
%jM_c: corridor frame max jerk [x,y,z]';
%__________________________________________________________________________

vM_c = zeros(3,1);
aM_c = zeros(3,1);
jM_c = zeros(3,1);
pitch = abs(pitch);

%---First we maximize the tilted rectangular area in the x-z surface.
K = sqrt(2)/2;
[vM_c(1), vM_c(3)] = tiltMaxRect(2*K*vM(1),2*vM(2),pitch);
[aM_c(1), aM_c(3)] = tiltMaxRect(2*K*aM(1),2*aM(2),pitch);
[jM_c(1), jM_c(3)] = tiltMaxRect(2*K*jM(1),2*jM(2),pitch);

%---Then find the max achieveable length in the y direction
vM_c(2) = sqrt(4*vM(1)^2-(vM_c(1)*cos(pitch)+vM_c(3)*sin(pitch))^2);
aM_c(2) = sqrt(4*aM(1)^2-(aM_c(1)*cos(pitch)+aM_c(3)*sin(pitch))^2);
jM_c(2) = sqrt(4*jM(1)^2-(jM_c(1)*cos(pitch)+jM_c(3)*sin(pitch))^2);

vM_c = vM_c/2;
aM_c = aM_c/2;
jM_c = jM_c/2;
end 