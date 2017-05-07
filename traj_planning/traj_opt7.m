function X = traj_opt7(path, total_time, ts)
% Minimum snap trajectory optimization
% by solving 8m linear equations. Assumes constant speed
% @author       Yiren Lu
% @email        luyiren [at] seas [dot] upenn [dot] edu
% 
% @input:       path                (m+1) by 3 planning trajectory
%               total_time          total time
% @output       X                   8mx3 solution vector, where
%                                   X(8*(k-1)+1:8*k,d) is the coefficients
%                                   of x_k(t), i.e., 
%                                   x_k(t)=X(8*(k-1)+1:8*k,d)'*[t^3;t^2,t,1]
%                                   where d \in [1,2,3] to indicate dimension xyz
path0 = path;
% generate the trajectory here, and decide the total_time
[m,n] = size(path0); % n == 3
% there we set m = m - 1 for convenience
m = m-1;
% there are now in total m+1 points in the path, which seperate the path into
% m subpaths.

% ts(k) = t_{k+1}, e.g. ts(1) = t_0,
% time planning according to the segment length

% ts(m+1) = t_m
% In 7rd order (minimum snap) trajectory optimization, for each subpath there are 
% 8 parameters
% x_k(t) = sum_{i=1}^8 c_{k,i-1}*t^{i-1} 
%    for k = 1..(m-1)

% X contains the parameters
X = zeros(8*m,n);
A = zeros(8*m, 8*m, n);
Y = zeros(8*m,n);

for i = 1:n
   A(:,:,i) = eye(8*m)*eps;
   % constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k, where p_k is a
   % waypoint
   % x_k(t) = sum_{i=1}^8 c_{k,i-1}*t^{i-1} 
   %    for k = 1..(m-1)
   % e.g.   x_1(t_1) = x_2(t_1) = p_1;
   % there are in total 2*(m-1) constraints
   idx = 1; % constraint counter
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [ts(k+1)^7, ts(k+1)^6, ts(k+1)^5, ts(k+1)^4, ts(k+1)^3, ts(k+1)^2, ts(k+1), 1];
       Y(idx,i) = path0(k+1,i);
       idx = idx + 1;
       A(idx, 8*(k)+1:8*(k+1), i) = [ts(k+1)^7, ts(k+1)^6, ts(k+1)^5, ts(k+1)^4, ts(k+1)^3, ts(k+1)^2, ts(k+1), 1];
       Y(idx,i) = path0(k+1,i);
       idx = idx + 1;
   end
   % constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
   % e.g.  \dot{x}_1(t_1) = \dot{x}_2(t_1)
   % there are in total m-1 constraints
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [7*ts(k+1)^6, 6*ts(k+1)^5, 5*ts(k+1)^4, 4*ts(k+1)^3, 3*ts(k+1)^2, 2*ts(k+1), 1, 0];
       A(idx, 8*(k)+1:8*(k+1), i) = -[7*ts(k+1)^6, 6*ts(k+1)^5, 5*ts(k+1)^4, 4*ts(k+1)^3, 3*ts(k+1)^2, 2*ts(k+1), 1, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
   end

   % constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
   % e.g. \ddot{x}_1(t_1) = \ddot{x}_2(t_1)
   % there are in total m-1 constraints
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [42*ts(k+1)^5, 30*ts(k+1)^4, 20*ts(k+1)^3, 12*ts(k+1)^2, 6*ts(k+1), 2, 0, 0];
       A(idx, 8*(k)+1:8*(k+1), i) = -[42*ts(k+1)^5, 30*ts(k+1)^4, 20*ts(k+1)^3, 12*ts(k+1)^2, 6*ts(k+1), 2, 0, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
   end

   % constraint 4: x^(3)_k(t_k) = x^(3)_{k+1}(t_k)
   % e.g. x^(3)_1(t_1) = x^(3)_2(t_1)
   % there are in total m-1 constraints
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [210*ts(k+1)^4, 120*ts(k+1)^3, 60*ts(k+1)^2, 24*ts(k+1), 6, 0, 0, 0];
       A(idx, 8*(k)+1:8*(k+1), i) = -[210*ts(k+1)^4, 120*ts(k+1)^3, 60*ts(k+1)^2, 24*ts(k+1), 6, 0, 0, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
   end

   % constraint 5: x^(4)_k(t_k) = x^(4)_{k+1}(t_k)
   % e.g. x^(4)_1(t_1) = x^(4)_2(t_1)
   % there are in total m-1 constraints
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [840*ts(k+1)^3, 360*ts(k+1)^2, 120*ts(k+1), 24, 0, 0, 0, 0];
       A(idx, 8*(k)+1:8*(k+1), i) = -[840*ts(k+1)^3, 360*ts(k+1)^2, 120*ts(k+1), 24, 0, 0, 0, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
   end

   % constraint 6: x^(5)_k(t_k) = x^(5)_{k+1}(t_k)
   % e.g. x^(5)_1(t_1) = x^(5)_2(t_1)
   % there are in total m-1 constraints
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [2520*ts(k+1)^2, 720*ts(k+1), 120, 0, 0, 0, 0, 0];
       A(idx, 8*(k)+1:8*(k+1), i) = -[2520*ts(k+1)^2, 720*ts(k+1), 120, 0, 0, 0, 0, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
   end

   % constraint 7: x^(6)_k(t_k) = x^(6)_{k+1}(t_k)
   % e.g. x^(6)_1(t_1) = x^(6)_2(t_1)
   % there are in total m-1 constraints
   for k = 1:(m-1)
       A(idx, 8*(k-1)+1:8*k, i) = [5040*ts(k+1), 720, 0, 0, 0, 0, 0, 0];
       A(idx, 8*(k)+1:8*(k+1), i) = -[5040*ts(k+1), 720, 0, 0, 0, 0, 0, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
   end

   % so far there are 8(m-1) constraints
   % there are 8 left:
   %    x_1(t_0) = p_0
   %    x^(1)_0(t_0) = 0
   %    x^(2)_0(t_0) = 0
   %    x^(3)_0(t_0) = 0
   %    x_T(t_T) = p_T
   %    x^(1)_T(t_T) = 0
   %    x^(2)_T(t_T) = 0
   %    x^(3)_T(t_T) = 0

   k = 1;
   A(idx, 8*(k-1)+1:8*k, i) = [ts(k)^7, ts(k)^6, ts(k)^5, ts(k)^4, ts(k)^3, ts(k)^2, ts(k), 1];
   Y(idx,i) = path0(k,i);
   idx = idx + 1;
   A(idx, 8*(k-1)+1:8*k, i) = [7*ts(k)^6, 6*ts(k)^5, 5*ts(k)^4, 4*ts(k)^3, 3*ts(k)^2, 2*ts(k), 1, 0];
   Y(idx,i) = 0;
   idx = idx + 1;
   A(idx, 8*(k-1)+1:8*k, i) = [42*ts(k)^5, 30*ts(k)^4, 20*ts(k)^3, 12*ts(k)^2, 6*ts(k), 2, 0, 0];
   Y(idx,i) = 0;
   idx = idx + 1;
   A(idx, 8*(k-1)+1:8*k, i) = [210*ts(k)^4, 120*ts(k)^3, 60*ts(k)^2, 24*ts(k), 6, 0, 0, 0];
   Y(idx,i) = 0;
   idx = idx + 1;

   k = m;
   A(idx, 8*(k-1)+1:8*k, i) = [ts(k+1)^7, ts(k+1)^6, ts(k+1)^5, ts(k+1)^4, ts(k+1)^3, ts(k+1)^2, ts(k+1), 1];
   Y(idx,i) = path0(k+1,i);
   idx = idx + 1;
   A(idx, 8*(k-1)+1:8*k, i) = [7*ts(k+1)^6, 6*ts(k+1)^5, 5*ts(k+1)^4, 4*ts(k+1)^3, 3*ts(k+1)^2, 2*ts(k+1), 1, 0];
   Y(idx,i) = 0;
   idx = idx + 1;
   A(idx, 8*(k-1)+1:8*k, i) = [42*ts(k+1)^5, 30*ts(k+1)^4, 20*ts(k+1)^3, 12*ts(k+1)^2, 6*ts(k+1), 2, 0, 0];
   Y(idx,i) = 0;
   idx = idx + 1;
   A(idx, 8*(k-1)+1:8*k, i) = [210*ts(k+1)^4, 120*ts(k+1)^3, 60*ts(k+1)^2, 24*ts(k+1), 6, 0, 0, 0];
   Y(idx,i) = 0;
   idx = idx + 1;
%    A(:,:,i) = A(:,:,i) + eye(8*m)*eps;
   X(:,i) = A(:,:,i)\Y(:,i);
end
