function collide = collision_check(p, margin)
collide = 0;
if(size(p,1) <= 1)
    return;
end
p(:,3) = p(:,3)/3; % scale z-axis by 3 to make it ellipsoid
dis = pdist(p);
if min(dis) < 2*margin
    collide = 1;
end
