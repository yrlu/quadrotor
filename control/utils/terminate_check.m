function [ terminate_cond ] = terminate_check( x, time, stop, pos_tol, vel_tol, time_tol)
%TERMINATE_CHECK Check termination criteria, including position, velocity and time
nquad = length(stop);

% Initialize
pos_check = true;
vel_check = true;
pos_col_check = zeros(nquad, 3);

% Check position and velocity and still time for each quad
for qn = 1:nquad
    pos_check = pos_check && (norm(x{qn}(1:3) - stop{qn}) < pos_tol);
    vel_check = vel_check && (norm(x{qn}(4:6)) < vel_tol);
    pos_col_check(qn,:) = x{qn}(1:3)';
end

% Check total simulation time
time_check = time > time_tol;
% Check collision criteria
col_check = collision_check(pos_col_check, 0.3);

if (pos_check && vel_check)
    terminate_cond = 1; % Robot reaches goal and stops, successful
elseif time_check
    terminate_cond = 2; % Robot doesn't reach goal within given time, not complete
elseif col_check
    terminate_cond = 3; % Robot collide with each other
else
    terminate_cond = 0;
end

end