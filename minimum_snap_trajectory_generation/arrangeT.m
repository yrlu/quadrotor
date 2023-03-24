function ts = arrangeT(waypts,T)
    x = waypts(:,2:end) - waypts(:,1:end-1);
    dist = sum(x.^2,1).^0.5;
    k = T/sum(dist);
    ts = [0 cumsum(dist*k)];
end