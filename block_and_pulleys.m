%Lecture 7
clear; clc; close all;

%%initial conditions
M = 2.0;            %arb mass of block
g = 9.81;           %gravity
D = 5.0;            %arb distance to pulley
L = 4.0;            %arb initial depth

%being able to edit tension so u can see what happens
% Minimum T to reach y=0 is: (g*L*M/2) / (sqrt(L^2 + D^2) - D) 
T_min = (g * L * M / 2) / (sqrt(L^2 + D^2) - D); 
T = T_min * 1;    %if u increase the multiplier it shoots up faster

t_max = 3.0;        %sim time
fps = 30;
tspan = linspace(0, t_max, t_max * fps);

%%ode45
% State vector z = [y; y_dot]
% dz/dt = [y_dot; acceleration]
% Accel formula: -g - (2*y*T) / (M * sqrt(y^2 + D^2)) 
ode_fun = @(t, z) [z(2); ...
                   -g - (2 * z(1) * T) / (M * sqrt(z(1)^2 + D^2))];

initial_conditions = [-L; 0]; % Starts at depth -L, zero velocity

[t, z] = ode45(ode_fun, tspan, initial_conditions);

y = z(:, 1);
y_dot = z(:, 2);

%%plot
figure('Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; axis equal; grid on;
axis([-D-1 D+1 -L-1 L+2]); % Adjusted axis to show "shooting up"
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Lifting a Block with Pulleys (T = %.1f N)', T));

%references
plot([-D D], [0 0], 'k--', 'LineWidth', 1); % Horizontal reference 
plot(-D, 0, 'ko', 'MarkerFaceColor', 'k');   % Wall attachment
plot(D, 0, 'ko', 'MarkerFaceColor', 'k');    % Pulley

%moving stuff
rope_L = line([0 0], [0 0], 'Color', 'b', 'LineWidth', 1.5);
rope_R = line([0 0], [0 0], 'Color', 'b', 'LineWidth', 1.5);
block = plot(0, 0, 'rs', 'MarkerSize', 20, 'MarkerFaceColor', 'r');

%%movie magic
for k = 1:length(t)
    % Update Block Position (stays on x=0) [cite: 209]
    set(block, 'XData', 0, 'YData', y(k));
    
    %updating rope positions
    set(rope_L, 'XData', [-D 0], 'YData', [0 y(k)]);
    set(rope_R, 'XData', [D 0], 'YData', [0 y(k)]);
    
    drawnow limitrate;
    pause(1/fps);
end
