function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
map3d = map.map3d_collision;
sz = size(map3d); nx = sz(1); ny = sz(2); nz = sz(3);
% vectorized
points = points_to_idx(map, points);
idx = (points(:,3)-1)*nx*ny + (points(:,2)-1)*nx + points(:,1);
C = map3d(idx) ~= 255 | map3d(nx*ny*nz+idx) ~= 255 | map3d(nx*ny*nz*2+idx) ~= 255;
end
