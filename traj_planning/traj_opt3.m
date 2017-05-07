function X = traj_opt3(path, total_time, ts)
% 5th order trajectory optimization (minimum acceleration)
% by solving 4m linear equations. Assumes constant speed
% @author       Yiren Lu
% @email        luyiren [at] seas [dot] upenn [dot] edu
% 
% @input:       path                (m+1) by 3 planning trajectory
%               total_time          total time
% @output       X                   4mx3 solution vector, where
%                                   X(4*(k-1)+1:4*k,d) is the coefficients
%                                   of x_k(t), i.e., 
%                                   x_k(t)=X(4*(k-1)+1:4*k,d)'*[t^3;t^2,t,1]
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
   % In 3rd order trajectory optimization, for each subpath there are 4
   % parameters
   % x_k(t) = c_{k,3}*t^3 + c_{k,2}*t^2 + c_{k,1}*t^1 + c_{k,0}
   %    for k = 1..(m-1)
   
   % X contains the parameters
   X = zeros(4*m,n);
   A = zeros(4*m, 4*m, n);
   Y = zeros(4*m,n);

   for i = 1:n
       A(:,:,i) = eye(4*m)*eps;
       % constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k, where p_k is a
       % waypoint
       % x_k(t) = c_{k,3}*t^3 + c_{k,2}*t^2 + c_{k,1}*t^1 + c_{k,0}
       %    for k = 1..(m-1)
       % e.g.   x_1(t_1) = x_2(t_1) = p_1;
       % there are in total 2*(m-1) constraints
       idx = 1; % constraint counter
       for k = 1:(m-1)
           A(idx, 4*(k-1)+1:4*k, i) = [ts(k+1)^3, ts(k+1)^2, ts(k+1), 1];
           Y(idx,i) = path0(k+1,i);
           idx = idx + 1;
           A(idx, 4*(k)+1:4*(k+1), i) = [ts(k+1)^3, ts(k+1)^2, ts(k+1), 1];
           Y(idx,i) = path0(k+1,i);
           idx = idx + 1;
       end
       % constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
       % \dot{x}_k(t) = 3*c_{k,3}*t^2 + 2*c_{k,2}*t + c_{k,1}
       % e.g.  \dot{x}_1(t_1) = \dot{x}_2(t_1)
       % there are in total m-1 constraints
       for k = 1:(m-1)
           A(idx, 4*(k-1)+1:4*k, i) = [3*ts(k+1)^2, 2*ts(k+1), 1, 0];
           A(idx, 4*(k)+1:4*(k+1), i) = -[3*ts(k+1)^2, 2*ts(k+1), 1, 0];
           Y(idx,i) = 0;
           idx = idx + 1;
       end
       
       % constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
       % \ddot{x}_k(t) =6*c_{k,3}*t + 2*c_{k,2}
       % e.g. \ddot{x}_1(t_1) = \ddot{x}_2(t_1)
       % there are in total m-1 constraints
       for k = 1:(m-1)
           A(idx, 4*(k-1)+1:4*k, i) = [6*ts(k+1), 2, 0, 0];
           A(idx, 4*(k)+1:4*(k+1), i) = -[6*ts(k+1), 2, 0, 0];
           Y(idx,i) = 0;
           idx = idx + 1;
       end
       
       % so far there are 2*(m-1) + (m-1) + (m-1) = 4m - 4 constraints
       % there are 4 left:
       %    x_1(t_0) = p_0
       %    x_T(t_T) = p_T
       %    \dot{x}_0(t_0) = 0
       %    \dot{x}_T(t_T) = 0
       k = 1;
       A(idx, 4*(k-1)+1:4*k, i) = [ts(k)^3, ts(k)^2, ts(k), 1];
       Y(idx,i) = path0(k,i);
       idx = idx + 1;
       A(idx, 4*(k-1)+1:4*k, i) = [3*ts(k)^2, 2*ts(k), 1, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
       k = m;
       A(idx, 4*(k-1)+1:4*k, i) = [ts(k+1)^3, ts(k+1)^2, ts(k+1), 1];
       Y(idx,i) = path0(k+1,i);
       idx = idx + 1;
       A(idx, 4*(k-1)+1:4*k, i) = [3*ts(k+1)^2, 2*ts(k+1), 1, 0];
       Y(idx,i) = 0;
       idx = idx + 1;
       % add regulerization to prevent singularity
       A(:,:,i) = A(:,:,i)+eye(4*m)*eps;
       X(:,i) = A(:,:,i)\Y(:,i);
   end

end