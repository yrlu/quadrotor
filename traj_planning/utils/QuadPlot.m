classdef QuadPlot < handle
    %QUADPLOT Visualization class for quad

    properties (SetAccess = public)
        k = 0;
        qn;             % quad number
        time = 0;       % time
        state;          % state
        des_state;      % desried state [x; y; z; xdot; ydot; zdot];
        rot;            % rotation matrix body to world

        color;          % color of quad
        wingspan;       % wingspan
        height;         % height of quad
        motor;          % motor position

        state_hist        % position history
        state_des_hist;   % desired position history
        time_hist;      % time history
        max_iter;       % max iteration
    end

    properties (SetAccess = private)
        h_3d
        h_m13;  % motor 1 and 3 handle
        h_m24;  % motor 2 and 4 handle
        h_qz;   % z axis of quad handle
        h_qn;   % quad number handle
        h_pos_hist;     % position history handle
        h_pos_des_hist; % desired position history handle
        text_dist;  % distance of quad number to quad
    end

    methods
        % Constructor
        function Q = QuadPlot(qn, state, wingspan, height, color, max_iter, h_3d)
            Q.qn = qn;
            Q.state = state;
            Q.wingspan = wingspan;
            Q.color = color;
            Q.height = height;
            Q.rot = QuatToRot(Q.state(7:10));
            Q.motor = quad_pos(Q.state(1:3), Q.rot, Q.wingspan, Q.height);
            Q.text_dist = Q.wingspan / 3;
            Q.des_state = Q.state(1:6);

            Q.max_iter = max_iter;
            Q.state_hist = zeros(6, max_iter);
            Q.state_des_hist = zeros(6, max_iter);
            Q.time_hist = zeros(1, max_iter);

            % Initialize plot handle
            if nargin < 7, h_3d = gca; end
            Q.h_3d = h_3d;
            hold(Q.h_3d, 'on')
            Q.h_pos_hist = plot3(Q.h_3d, Q.state(1), Q.state(2), Q.state(3), 'r.');
            Q.h_pos_des_hist = plot3(Q.h_3d, Q.des_state(1), Q.des_state(2), Q.des_state(3), 'b.');
            Q.h_m13 = plot3(Q.h_3d, ...
                Q.motor(1,[1 3]), ...
                Q.motor(2,[1 3]), ...
                Q.motor(3,[1 3]), ...
                '-ko', 'MarkerFaceColor', Q.color, 'MarkerSize', 5);
            Q.h_m24 = plot3(Q.h_3d, ...
                Q.motor(1,[2 4]), ...
                Q.motor(2,[2 4]), ...
                Q.motor(3,[2 4]), ...
                '-ko', 'MarkerFaceColor', Q.color, 'MarkerSize', 5);
            Q.h_qz = plot3(Q.h_3d, ...
                Q.motor(1,[5 6]), ...
                Q.motor(2,[5 6]), ...
                Q.motor(3,[5 6]), ...
                'Color', Q.color, 'LineWidth', 2);
            Q.h_qn = text(...
                Q.motor(1,5) + Q.text_dist, ...
                Q.motor(2,5) + Q.text_dist, ...
                Q.motor(3,5) + Q.text_dist, num2str(qn));
            hold(Q.h_3d, 'off')
        end

        % Update quad state
        function UpdateQuadState(Q, state, time)
            Q.state = state;
            Q.time = time;
            Q.rot = QuatToRot(state(7:10))'; % Q.rot needs to be body-to-world
        end

        % Update desired quad state
        function UpdateDesiredQuadState(Q, des_state)
            Q.des_state = des_state;
        end

        % Update quad history
        function UpdateQuadHist(Q)
            Q.k = Q.k + 1;
            Q.time_hist(Q.k) = Q.time;
            Q.state_hist(:,Q.k) = Q.state(1:6);
            Q.state_des_hist(:,Q.k) = Q.des_state(1:6);
        end

        % Update motor position
        function UpdateMotorPos(Q)
            Q.motor = quad_pos(Q.state(1:3), Q.rot, Q.wingspan, Q.height);
        end

        % Truncate history
        function TruncateHist(Q)
            Q.time_hist = Q.time_hist(1:Q.k);
            Q.state_hist = Q.state_hist(:, 1:Q.k);
            Q.state_des_hist = Q.state_des_hist(:, 1:Q.k);
        end

        % Update quad plot
        function UpdateQuadPlot(Q, state, des_state, time)
            Q.UpdateQuadState(state, time);
            Q.UpdateDesiredQuadState(des_state);
            Q.UpdateQuadHist();
            Q.UpdateMotorPos();
            set(Q.h_m13, ...
                'XData', Q.motor(1,[1 3]), ...
                'YData', Q.motor(2,[1 3]), ...
                'ZData', Q.motor(3,[1 3]));
            set(Q.h_m24, ...
                'XData', Q.motor(1,[2 4]), ...
                'YData', Q.motor(2,[2 4]), ...
                'ZData', Q.motor(3,[2 4]));
            set(Q.h_qz, ...
                'XData', Q.motor(1,[5 6]), ...
                'YData', Q.motor(2,[5 6]), ...
                'ZData', Q.motor(3,[5 6]))
            set(Q.h_qn, 'Position', ...
                [Q.motor(1,5) + Q.text_dist, ...
                Q.motor(2,5) + Q.text_dist, ...
                Q.motor(3,5) + Q.text_dist]);
            set(Q.h_pos_hist, ...
                'XData', Q.state_hist(1,1:Q.k), ...
                'YData', Q.state_hist(2,1:Q.k), ...
                'ZData', Q.state_hist(3,1:Q.k));
            set(Q.h_pos_des_hist, ...
                'XData', Q.state_des_hist(1,1:Q.k), ...
                'YData', Q.state_des_hist(2,1:Q.k), ...
                'ZData', Q.state_des_hist(3,1:Q.k));
            drawnow;
        end
    end

end
