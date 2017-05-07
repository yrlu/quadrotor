function path1 = simplify_path2(map, path)
path0 = path;
if size(path,1) == 0
path1 = path;
return;
end
keep_pts = logical(zeros(size(path,1),1));
keep_pts(1) = 1;
keep_pts(end) = 1;
last_keep = path(1,:);

function c = check_line_collision(map, start_pos, end_pos)
    % generate intermedia points
    lenxy = sqrt(sum((start_pos(1:2) - end_pos(1:2)).^2));
    lenz = sqrt(sum((start_pos(3) - end_pos(3)).^2));
    n_pts = int32(max(lenxy/map.xy_res, lenz/map.z_res)) + 2;
    pts = [linspace(start_pos(1), end_pos(1), n_pts);
           linspace(start_pos(2), end_pos(2), n_pts);
           linspace(start_pos(3), end_pos(3), n_pts)]';
    c = collide(map, pts);
    c = any(c);
end

counter = 1;
for i = 2:size(path,1)-1
    counter = counter + 1;
    if (~check_line_collision(map, last_keep, path(i,:)) & check_line_collision(map, last_keep, path(i+1,:))) | counter == 200000
        count = 1;
        last_keep = path(i,:);
        keep_pts(i) = 1;
    end
end
path = path(keep_pts,:);

% add intermedia points to avoid running out of the map
path1 = [];
for i = 1:size(path,1)-1
    path1 = [path1; path(i,:)];
%     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.1];
%     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.2];
%     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.5];
%     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.7];
%     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.9];
end
path1 = [path1; path(end,:)];
end