function demo2_minimum_snap_corridor()
clear,clc;

%% condition
waypts = [0,0;
          1,2;
          2,0;
          4,5;
          5,2]';
v0 = [0,0];
a0 = [0,0];
v1 = [0,0];
a1 = [0,0];
T = 5;
n_order = 5;

%% sample mid points
r = 0.2;  %% corridor r
step = r;
new_waypts = waypts(:,1);
for i=2:size(waypts,2)
    x1 = waypts(1,i-1);
    y1 = waypts(2,i-1);
    x2 = waypts(1,i);
    y2 = waypts(2,i);
    n = ceil(hypot(x1-x2,y1-y2)/step)+1;
    sample_pts = [linspace(x1,x2,n);linspace(y1,y2,n)];
    new_waypts = [new_waypts sample_pts(:,2:end)];
end
ts = arrangeT(new_waypts,T);
    
%% trajectory plan
polys_x = minimum_snap_single_axis_corridor(new_waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1),r);
polys_y = minimum_snap_single_axis_corridor(new_waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2),r);

%% result show
figure(1)
% plot(waypts(1,:),waypts(2,:),'c','LineWidth',30);hold on;
plot(new_waypts(1,:),new_waypts(2,:),'.g');hold on;
plot(waypts(1,:),waypts(2,:),'*r');hold on;
% plot(waypts(1,:),waypts(2,:),'--b');
for i=1:size(new_waypts,2)
    plot_rect(new_waypts(:,i),r);
end

title('minimum snap trajectory with corridor');
tt = 0:0.01:T;
xx = polys_vals(polys_x,ts,tt,0);
yy = polys_vals(polys_y,ts,tt,0);
plot(xx,yy,'r');

end

function plot_rect(center,r)
p1 = center+[-r;-r];
p2 = center+[-r;r];
p3 = center+[r;r];
p4 = center+[r;-r];
plot_line(p1,p2);
plot_line(p2,p3);
plot_line(p3,p4);
plot_line(p4,p1);
end

function plot_line(p1,p2)
a = [p1(:),p2(:)];    
plot(a(1,:),a(2,:),'b');
end

function polys = minimum_snap_single_axis_corridor(waypts,ts,n_order,v0,a0,ve,ae,corridor_r)
p0 = waypts(1);
pe = waypts(end);

n_poly = length(waypts)-1;
n_coef = n_order+1;

% compute Q
Q_all = [];
for i=1:n_poly
    Q_all = blkdiag(Q_all,computeQ(n_order,3,ts(i),ts(i+1)));
end
b_all = zeros(size(Q_all,1),1);

Aeq = zeros(3*n_poly+3,n_coef*n_poly);
beq = zeros(3*n_poly+3,1);

% start/terminal pva constraints  (6 equations)
Aeq(1:3,1:n_coef) = [calc_tvec(ts(1),n_order,0);
                     calc_tvec(ts(1),n_order,1);
                     calc_tvec(ts(1),n_order,2)];
Aeq(4:6,n_coef*(n_poly-1)+1:n_coef*n_poly) = ...
                    [calc_tvec(ts(end),n_order,0);
                     calc_tvec(ts(end),n_order,1);
                     calc_tvec(ts(end),n_order,2)];
beq(1:6,1) = [p0,v0,a0,pe,ve,ae]';
neq = 6;

% continuous constraints  ((n_poly-1)*3 equations)
for i=1:n_poly-1
    tvec_p = calc_tvec(ts(i+1),n_order,0);
    tvec_v = calc_tvec(ts(i+1),n_order,1);
    tvec_a = calc_tvec(ts(i+1),n_order,2);
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p,-tvec_p];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v,-tvec_v];
    neq=neq+1;
    Aeq(neq,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a,-tvec_a];
end

% corridor constraints (n_ploy-1 iequations)
Aieq = zeros(2*(n_poly-1),n_coef*n_poly);
bieq = zeros(2*(n_poly-1),1);
for i=1:n_poly-1
    tvec_p = calc_tvec(ts(i+1),n_order,0);
    Aieq(2*i-1:2*i,n_coef*i+1:n_coef*(i+1)) = [tvec_p;-tvec_p];
    bieq(2*i-1:2*i) = [waypts(i+1)+corridor_r corridor_r-waypts(i+1)];
end

p = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq);

polys = reshape(p,n_coef,n_poly);

end

