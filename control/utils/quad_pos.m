function [ quad ] = quad_pos( pos, rot, L, H )
%QUAD_POS Calculates coordinates of quadrotor's position in world frame
% pos       3x1 position vector [x; y; z];
% rot       3x3 body-to-world rotation matrix
% L         1x1 length of the quad

if nargin < 4; H = 0.05; end

% wRb   = RPYtoRot_ZXY(euler(1), euler(2), euler(3))';
wHb   = [rot pos(:); 0 0 0 1]; % homogeneous transformation from body to world

quadBodyFrame  = [L 0 0 1; 0 L 0 1; -L 0 0 1; 0 -L 0 1; 0 0 0 1; 0 0 H 1]';
quadWorldFrame = wHb * quadBodyFrame;
quad           = quadWorldFrame(1:3, :);

end
