function params = crazyflie()
% crazyflie: physical parameters for the Crazyflie 2.0
%
% 2016 Bernd Pfrommer
%
% This function creates a struct with the basic parameters for the
% Crazyflie 2.0 quad rotor (without camera, but with about 5 vicon
% markers)
%
% Model assumptions based on physical measurements:
%
% motor + mount + vicon marker = mass point of 3g
% arm length of mass point: 0.046m from center
% battery pack + main board are combined into cuboid (mass 18g) of
% dimensions:
%
%   width  = 0.03m
%   depth  = 0.03m
%   height = 0.012m
%

m = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
g = 9.81;   % gravitational constant
I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];
L = 0.046; % arm length in m

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = L;

params.maxangle = 40*pi/180; % you can specify the maximum commanded angle here
params.maxF     = 2.5*m*g;   % left these untouched from the nano plus
params.minF     = 0.05*m*g;  % left these untouched from the nano plus

% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m

end
