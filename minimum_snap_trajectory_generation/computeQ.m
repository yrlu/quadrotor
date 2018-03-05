% n:polynormial order
% r:derivertive order, 1:minimum vel 2:minimum acc 3:minimum jerk 4:minimum snap
% t1:start timestamp for polynormial
% t2:end timestap for polynormial
function Q = computeQ(n,r,t1,t2)
T = zeros((n-r)*2+1,1);
for i = 1:(n-r)*2+1
    T(i) = t2^i-t1^i;
end
Q = zeros(n);
for i = r+1:n+1
    for j = i:n+1
        k1 = i-r-1;
        k2 = j-r-1;
        k = k1+k2+1;
        Q(i,j) = prod(k1+1:k1+r)*prod(k2+1:k2+r)/k*T(k);
        Q(j,i) = Q(i,j);
    end
end