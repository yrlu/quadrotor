function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

figure(1);
% path = points_to_idx(map, path);

[x,y,z] = meshgrid(1:size(map.map3d,1),1:size(map.map3d,2),1:size(map.map3d,3));
x = x(:); y = y(:); z = z(:);

xyz = [];
for i = 1:numel(x)
    if map.map3d(x(i),y(i),z(i),1)~= 255 | map.map3d(x(i),y(i),z(i),2)~= 255 | map.map3d(x(i),y(i),z(i),2)~= 255
        xyz = [xyz; x(i),y(i),z(i)];
    end
end
rgb = zeros(size(xyz,1),3);
for i = 1:size(xyz,1)
    rgb(i,:) = map.map3d(xyz(i,1),xyz(i,2),xyz(i,3),:);
end
xyz = idx_to_points(map, xyz);
if size(xyz,1) > 0
pcshow(xyz, rgb, 'MarkerSize', 20);
end
hold on;
if size(path,1) > 0
pcshow(path, [0,0,0],'MarkerSize', 20);
end
hold off;
end