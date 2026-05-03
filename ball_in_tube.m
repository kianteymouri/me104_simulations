% ME 104 Lecture 6: Ball Dynamics Animation
clear; clc; close all;

%%initial conditions
m = 1.0;          %arb mass
omega = 2.0;      %arb cnst angular velocity
cs = 0.5;         %arb air drag coeff
l0 = 1.0;         %arb initial distance
t_max = 4.5;      
fps = 30;         %movie magic jargon
t = linspace(0, t_max, t_max * fps);

%% solving for r(t)
% Characteristic equation roots
term1 = -cs / (2*m);
term2 = sqrt(cs^2 + 4 * m^2 * omega^2) / (2*m); 
b1 = term1 + term2; 
b2 = term1 - term2; 

%radial position
r = (l0 / (b2 - b1)) * (b2 * exp(b1 * t) - b1 * exp(b2 * t));
theta = omega * t; % [cite: 13]

x = r .* cos(theta);
y = r .* sin(theta);

%%plot
figure('Color', 'w', 'Position', [100, 100, 800, 800]);
hold on; axis equal; grid on;

%axis
axis([-20 20 -20 20]);

%reference ground plane
line([-20 20], [0 0], 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LineStyle', '--');
text(-19, -1, 'Reference Plane (Ground)');

%intiializing ball
ball = plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
trail = plot(nan, nan, 'r:', 'LineWidth', 1.5);

title('Ball Flinging out of Frictionless Tube');
xlabel('x (m)'); ylabel('y (m)');

%% movie magic
for k = 1:length(t)
    % ball and trail
    set(ball, 'XData', x(k), 'YData', y(k));
    set(trail, 'XData', x(1:k), 'YData', y(1:k));
    
    %stop if ball hits ground
    if y(k) < -0.1 && k > 1 
        break; 
    end
    
    if r(k) > 20
        break;
    end
    
    drawnow limitrate;
    pause(1/fps);
end
