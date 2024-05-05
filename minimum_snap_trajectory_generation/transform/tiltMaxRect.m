function [wr, hr] = tiltMaxRect(w,h,angle)
%Given a rectangle of size w x h, rotate it by angle, coumputes the width
%and height of the largetst possible axis-aligned rectangle inside it.
%Implemented from: https://stackoverflow.com/questions/16702966/rotate-image-and-crop-out-black-borders/16778797#16778797
%---Input:
%w: width of the outside rectangle
%h: height of the outside rectangle
%angle: angle
%---Output:
%wr: width of the inside rectangle
%hr: height of the inside rectangle
wr = 0;
hr = 0;

if(w <=0 || h<=0)
    return;
end

width_is_longer = (w>=h);
if(width_is_longer)
    side_long = w;
    side_short = h;
else
    side_long = h;
    side_short = w;
end

% since the solutions for angle, -angle and 180-angle are all the same,
% if suffices to look at the first quadrant and the absolute values of sin,cos:
sin_a = abs(sin(angle));
cos_a = abs(cos(angle));

if(side_short <= 2.0*sin_a*cos_a*side_long)
% half constrained case: two crop corners touch the longer side,
% the other two corners are on the mid-line parallel to the longer line
    x = 0.5*side_short;
    if(width_is_longer)
        wr = x/sin_a;
        hr = x/cos_a;
    else
        wr = x/cos_a;
        hr = x/sin_a;
    end
else
% fully constrained case: crop touches all 4 sides
    cos_2a = cos_a*cos_a - sin_a*sin_a;
    wr = (w*cos_a - h*sin_a)/cos_2a;
    hr = (h*cos_a - w*sin_a)/cos_2a;
end