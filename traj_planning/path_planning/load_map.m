function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
fid = fopen(filename);
tline = fgets(fid);
blocks = [];
while ischar(tline)
    % skip empty line and comments
    if numel(tline) > 1 & tline(1)~='#'
        % convert char array to string
        if strncmpi(tline, 'boundary', 8)
            boundary = strread(tline(9:end));
        end
        if strncmpi(tline, 'block', 5)
            block = strread(tline(6:end));
            assert(size(block,2) == 9);
            blocks = [blocks; block];
        end
    end
    tline = fgets(fid);
end
map.boundary = boundary;
map.blocks = blocks;
map.xy_res = xy_res;
map.z_res = z_res;
% map.margin = margin + 0.2; enforce the path finder to pick a safe path.
map.margin = margin;
n_x = ceil(abs(boundary(4) - boundary(1))/xy_res);
n_y = ceil(abs(boundary(5) - boundary(2))/xy_res);
n_z = ceil(abs(boundary(6) - boundary(3))/z_res);
map.nx = n_x;
map.ny = n_y;
map.nz = n_z;

map3d = ones(n_x, n_y, n_z, 3)*255;
occ_map = logical(zeros(n_x, n_y, n_z));
for i = 1:size(blocks,1)
    block = blocks(i, :);
   	xyzs = points_to_idx(map, [block(1) - map.margin, block(2) - map.margin, block(3) - map.margin]);
    xs = xyzs(1); ys = xyzs(2); zs = xyzs(3);
    xyze = points_to_idx(map, [block(4) + map.margin, block(5) + map.margin, block(6) + map.margin]);
    xe = xyze(1); ye = xyze(2); ze = xyze(3);
    % convert block to coordinates
    map3d(xs:xe,ys:ye,zs:ze,1) = uint8(block(7));
    map3d(xs:xe,ys:ye,zs:ze,2) = uint8(block(8));
    map3d(xs:xe,ys:ye,zs:ze,3) = uint8(block(9));
    occ_map(xs:xe,ys:ye,zs:ze) = 1;
end
map.map3d = map3d;

map3d_collision = ones(n_x, n_y, n_z, 3)*255;
for i = 1:size(blocks,1)
    block = blocks(i, :);
   	xyzs = points_to_idx(map, [block(1) - margin, block(2) - margin, block(3) - margin]);
    xs = xyzs(1); ys = xyzs(2); zs = xyzs(3);
    xyze = points_to_idx(map, [block(4) + margin, block(5) + margin, block(6) + margin]);
    xe = xyze(1); ye = xyze(2); ze = xyze(3);
    % convert block to coordinates
    map3d_collision(xs:xe,ys:ye,zs:ze,1) = uint8(block(7));
    map3d_collision(xs:xe,ys:ye,zs:ze,2) = uint8(block(8));
    map3d_collision(xs:xe,ys:ye,zs:ze,3) = uint8(block(9));
end
map.map3d_collision = map3d_collision;

map.occ_map = occ_map;
end
