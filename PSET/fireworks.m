%%pset 1 - problem 5: fireworks 

clc; clear; close all;

%%intial coniditions
c_range = [0.001, 0.01]; 
style_range = {'r', 'k'}; 
v = 10; 
g = 9.81; 
t0 = 0; tf = 1.5; 
m = 10/1000; 
x0 = 0; y0 = 0; 
num_traj = 8; 
theta0_range = linspace(45, 135, num_traj);

% Pre-calculate all trajectories first
all_data = cell(2, num_traj);
t_common = linspace(t0, tf, 100);

for drag_idx = 1:2
    c = c_range(drag_idx);
    % Derivative function: r = [x, y, vx, vy]
    % Equations: ax = -c/m * vx * speed, ay = -c/m * vy * speed - g 
    D = @(t, r) [r(3); r(4); ...
                 -c/m*r(3)*sqrt(r(3)^2 + r(4)^2); ...
                 -c/m*r(4)*sqrt(r(3)^2 + r(4)^2) - g];
             
    for traj_idx = 1:num_traj
        theta0 = theta0_range(traj_idx);
        v_x0 = v*cosd(theta0);
        v_y0 = v*sind(theta0);
        r0 = [x0; y0; v_x0; v_y0];
        
        % Solve and interpolate to common time steps for the movie
        sol = ode45(D, [t0, tf], r0); 
        all_data{drag_idx, traj_idx} = deval(sol, t_common);
    end
end

%%movie magic
figure('Color', 'w');
hold on; grid on;
axis([-10 10 -2 6]); % Set fixed axis for the movie
xlabel('x (m)'); ylabel('y (m)');
title('Projectile Trajectories with Inertial Drag');

% Initialize plot handles
% Initializes an array to hold the plot handles for all 16 trajectories
handles = gobjects(2 * num_traj, 1);
count = 1;
for d = 1:2
    for t = 1:num_traj
        handles(count) = plot(nan, nan, style_range{d});
        count = count + 1;
    end
end

% Video Writer Setup
v_writer = VideoWriter('Projectile_Movie.mp4', 'MPEG-4');
v_writer.FrameRate = 30;
open(v_writer);

% Animation Loop
for frame = 1:length(t_common)
    count = 1;
    for d = 1:2
        for traj = 1:num_traj
            data = all_data{d, traj};
            % Update trajectory up to current frame
            set(handles(count), 'XData', data(1, 1:frame), 'YData', data(2, 1:frame));
            count = count + 1;
        end
    end
    drawnow limitrate;
    writeVideo(v_writer, getframe(gcf));
end

close(v_writer);
fprintf('Movie saved as Projectile_Movie.mp4\n');
