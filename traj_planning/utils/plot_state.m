function [ h_fig ] = plot_state( h_fig, state, time, name, type, view )
%PLOT_STATE visualize state data

if nargin < 6, view = 'sep'; end
if nargin < 5, type = 'vic'; end
if nargin < 4, name = 'pos'; end
if isempty(h_fig), h_fig = figure(); end
line_width = 2;

switch type
    case 'vic'
        line_color = 'r';
    case 'des'
        line_color = 'b';
    case 'est'
        line_color = 'g';
end

switch name
    case 'pos'
        labels = {'x [m]', 'y [m]', 'z [m]'};
    case 'vel'
        labels = {'xdot [m/s]', 'ydot [m/s]', 'zdot [m/s]'};
    case 'euler'
        labels = {'roll [rad]', 'pitch [rad]', 'yaw [rad]'};
end

figure(h_fig)
if strcmp(view, 'sep')
    % Plot seperate

    for i = 1:3
        subplot(3, 1, i)
        hold on
        plot(time, state(i,:), line_color, 'LineWidth', line_width);
        hold off
        xlim([time(1), time(end)])
        grid on
        xlabel('time [s]')
        ylabel(labels{i})
    end
elseif strcmp(view, '3d')
    % Plot 3d
    hold on
    plot3(state(1,:), state(2,:), state(3,:), line_color, 'LineWidth', line_width)
    hold off
    grid on
    xlabel(labels{1});
    ylabel(labels{2});
    zlabel(labels{3});
end

end
