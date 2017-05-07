function [ts, total_time] = generate_ts(path)
speed = 2;
path_len = sum(sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)));
total_time = path_len/speed;
% ts = linspace(0, total_time, size(path,1));
path_seg_len = sqrt(sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)));
ts = cumsum(path_seg_len);
ts = ts/ts(end);
ts = [0; ts]';
ts = ts*total_time;
end