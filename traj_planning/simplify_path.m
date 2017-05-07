function path1 = simplify_path(path)
% path1 = [path(1,:);path(1:4:end, :);path(end,:)];
% return
path0 = path;
% remove duplicated points:
rm_pts = logical(zeros(size(path,1),1));
for i = 1:size(path,1)-1
    if norm(path(i+1,:) - path(i,:)) < 1e-3
        rm_pts(i) = 1;
    end
end
path = path(~rm_pts,:);

% handle diagional case
path = path(1:2:end, :);

% check collinear
rm_pts = logical(zeros(size(path,1),1));
for i = 1:size(path,1) - 2
    if sum(sum((abs(bsxfun(@minus, path(i:i+2,:), mean(path(i:i+2,:))))<0.0001))) >= 6
        rm_pts(i+1) = 1;
    end
end
path = path(~rm_pts,:);

% add start/goal points
if any(path(1,:) ~= path0(1,:))
    path = [path0(1,:); path];
end
if any(path(end,:) ~= path0(end,:))
    path = [path; path0(end,:)];
end
path1 = path;
% plot_path(map, path1);
end