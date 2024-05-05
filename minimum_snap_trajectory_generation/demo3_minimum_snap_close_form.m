function demo3_minimum_snap_close_form()
clear,clc;
addpath estimate transform
%% condition
pf = [-1.5 1.5 1; 1.8 2 1.5; 1.8 -4 1.5;1 6 0.5; 1.5 0 1.5; -1.5 1.5 0.5;]*3;

v0 = [0,0,0];
a0 = [0,0,0];
j0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];
j1 = [0,0,0];

mV = [5 2];
mA = [3 2];
%% estimate and insert
%---Set up line to be fitted
data_size = size(pf,1);

Ts = 0;
for i = 1:data_size-1
    TM = shortestTime_synced(pf(i,:),pf(i+1,:),mV,mA)*1.2;
    Ts = [Ts; Ts(end)+TM];
end
T = Ts(end);

%---Set up the points to be fitted
insert_dist = 8;
Ds = [];
Dt = [];
for i=1:size(pf,1)-1
    seg_s=[];
    n_sample = 2 + round(norm(pf(i,:)-pf(i+1,:))/insert_dist);
    for j=1:3
        seg_s(j,:) = linspace(pf(i,j),pf(i+1,j),n_sample);
    end
    seg_t = linspace(Ts(i),Ts(i+1),n_sample);
    seg_s(:,end)=[];
    seg_t(end)=[];
    Ds = [Ds;seg_s'];
    Dt = [Dt;seg_t'];
end
Ds = [Ds;pf(end,:)];
Dt = [Dt;Ts(end)];
%%
waypts = Ds';
ts = Dt';
n_order=7;
%% trajectory plan
polys_x = minimum_snap_single_axis_close_form(waypts(1,:),ts,n_order,v0(1),a0(1),j0(1),v1(1),a1(1),j1(1));
polys_y = minimum_snap_single_axis_close_form(waypts(2,:),ts,n_order,v0(2),a0(2),j0(2),v1(2),a1(2),j1(2));
polys_z = minimum_snap_single_axis_close_form(waypts(3,:),ts,n_order,v0(3),a0(2),j0(3),v1(3),a1(3),j1(3));

%% result show
figure(1)
% plot(waypts(1,:),waypts(2,:),'*r');hold on;
plot3(waypts(1,:),waypts(2,:),waypts(3,:),'b*-');hold on;axis equal;
title('minimum snap trajectory');
color = ['grc'];
A = [];
% for i=1:size(polys_x,2)
    tt = 0:0.01:Dt(end);
    xx = polys_vals(polys_x,ts,tt,0);
    yy = polys_vals(polys_y,ts,tt,0);
    zz = polys_vals(polys_z,ts,tt,0);
    A =[A [polys_vals(polys_x,ts,tt,2);polys_vals(polys_y,ts,tt,2);polys_vals(polys_z,ts,tt,2);]];
    plot3(xx,yy,zz,color(mod(i,3)+1));
% end
figure(2);plot(A');

end

function polys = minimum_snap_single_axis_close_form(wayp,ts,n_order,v0,a0,j0,v1,a1,j1)
n_coef = n_order+1;
n_poly = length(wayp)-1;
% compute Q
Q_all = [];
for i=1:n_poly
    Q_all = blkdiag(Q_all,computeQ(n_order,4,ts(i),ts(i+1)));
end

% compute Tk   Tk(i,j) = ts(i)^(j-1)
tk = zeros(n_poly+1,n_coef);
for i = 1:n_coef
    tk(:,i) = ts(:).^(i-1);
end

% compute A (n_continuous*2*n_poly) * (n_coef*n_poly)
n_continuous = 4;  % 1:p  2:pv  3:pva  4:pvaj  5:pvajs
A = zeros(n_continuous*2*n_poly,n_coef*n_poly);
for i = 1:n_poly
    for j = 1:n_continuous
        for k = j:n_coef
            if k==j
                t1 = 1;
                t2 = 1;
            else %k>j
                t1 = tk(i,k-j+1);
                t2 = tk(i+1,k-j+1);
            end
            A(n_continuous*2*(i-1)+j,n_coef*(i-1)+k) = prod(k-j+1:k-1)*t1;
            A(n_continuous*2*(i-1)+n_continuous+j,n_coef*(i-1)+k) = prod(k-j+1:k-1)*t2;
        end
    end
end

% compute M
M = zeros(n_poly*2*n_continuous,n_continuous*(n_poly+1));
for i = 1:n_poly*2
    j = floor(i/2)+1;
    rbeg = n_continuous*(i-1)+1;
    cbeg = n_continuous*(j-1)+1;
    M(rbeg:rbeg+n_continuous-1,cbeg:cbeg+n_continuous-1) = eye(n_continuous);
end

% compute C
num_d = n_continuous*(n_poly+1);
C = eye(num_d);
df = [wayp,v0,a0,j0,v1,a1,j1]';% fix all pos(n_poly+1) + start va(2) + end va(2) 
fix_idx = [1:4:num_d,2,3,4,num_d-2,num_d-1,num_d];
free_idx = setdiff(1:num_d,fix_idx);
C = [C(:,fix_idx) C(:,free_idx)];

AiMC = inv(A)*M*C;
R = AiMC'*Q_all*AiMC;

n_fix = length(fix_idx);
Rff = R(1:n_fix,1:n_fix);
Rpp = R(n_fix+1:end,n_fix+1:end);
Rfp = R(1:n_fix,n_fix+1:end);
Rpf = R(n_fix+1:end,1:n_fix);

dp = -inv(Rpp)*Rfp'*df;

p = AiMC*[df;dp];

polys = reshape(p,n_coef,n_poly);

end

