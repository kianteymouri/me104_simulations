%%Lecture 14

clear; clc; close all;

%%parameters and constraints
m = 2.0;            % Mass (kg)
R = 1.5;            % Length of rigid bar (m)
k = 50;             % Spring constant (N/m)
l0 = 0.5;           % Spring neutral length (m)
T = 15;             % Constant Rope Tension (N)

%fixed positions of stuff
r0 = [-1.0; 1.0];   % Spring attachment point (Fixed)
r1 = [2.0; 2.0];    % Pulley location (Fixed)

%timing
target_fps = 60;                    % Target 60 Frames Per Second
t_final = 10;                        % Total duration in seconds
tspan = 0:(1/target_fps):t_final;    % Dense time vector for smoothness

%angles
theta_i = deg2rad(10);  % Starts stationary at theta_i
theta_f = deg2rad(180);  % Final target angle

%% dynamics solver
% State vector: [theta; theta_dot]
ode_fun = @(t, state) dynamics_fun(t, state, m, R, k, l0, T, r0, r1);
[t, sol] = ode45(ode_fun, tspan, [theta_i; 0]);

theta_vals = sol(:,1);
X_mass = R * cos(theta_vals);
Y_mass = R * sin(theta_vals);

%%visual setup
figure('Color', 'w', 'DoubleBuffer', 'on');
hold on; axis equal; grid on;
axis([-2 3 -1 3]);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('High-FPS Simulation (Target: %d Hz)', target_fps));

%static elements
plot(0, 0, 'ko', 'MarkerFaceColor', 'k');   % Pivot O
plot(r0(1), r0(2), 'k^', 'MarkerSize', 10); % Spring Anchor
plot(r1(1), r1(2), 'ko', 'MarkerSize', 12); % Pulley

%intiializing dynamics handles
hBar = line([0 0], [0 0], 'Color', 'k', 'LineWidth', 4);
hSpring = line([0 0], [0 0], 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
hRope = line([0 0], [0 0], 'Color', 'b', 'LineWidth', 1.5);
hMass = plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);

%% movie magic
for i = 1:length(t)
    set(hBar, 'XData', [0 X_mass(i)], 'YData', [0 Y_mass(i)]);
    set(hMass, 'XData', X_mass(i), 'YData', Y_mass(i));
    set(hSpring, 'XData', [r0(1) X_mass(i)], 'YData', [r0(2) Y_mass(i)]);
    set(hRope, 'XData', [X_mass(i) r1(1) r1(1)+0.5], ...
               'YData', [Y_mass(i) r1(2) r1(2)-0.5]);
    
    % stop if passing final angle
    if theta_vals(i) >= theta_f
        break;
    end
    
    drawnow; 
end

%%torque balance
function dstate = dynamics_fun(~, state, m, R, k, l0, T, r0, r1)
    th = state(1);
    dth = state(2);
    
    % position vector calc
    r_vec = [R*cos(th); R*sin(th)];
    et = [-sin(th); cos(th)]; % Tangential unit vector
    
    %spring forces
    l_vec = r_vec - r0;
    Fs = -k * (norm(l_vec) - l0) * (l_vec / norm(l_vec));
    
    %tension force
    rope_vec = r1 - r_vec;
    FT = T * (rope_vec / norm(rope_vec));
    
    % torque abt pivot
    torque = dot(Fs, et)*R + dot(FT, et)*R;
    alpha = torque / (m * R^2); % I = m*R^2 for a point mass
    
    dstate = [dth; alpha];
end
