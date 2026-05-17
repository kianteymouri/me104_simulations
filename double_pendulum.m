function double_pendulum_simulation()
    % ME 104 Lecture 40: Double Pendulum Simulation
    % Derived explicitly using the Euler-Lagrange equations: d/dt(dL/da_dot) - dL/da = 0
    clear all; close all; clc;

    %% --- PARAMETER CONFIGURATION ZONE ---
    % Physical parameters of the system
    params.m1 = 1.0;   % Mass of ball 1 (kg)
    params.m2 = 1.0;   % Mass of ball 2 (kg)
    params.l1 = 1.5;   % Length of rod 1 (m)
    params.l2 = 1.2;   % Length of rod 2 (m)
    params.g  = 9.81;  % Acceleration due to gravity (m/s^2)

    % Initial Conditions: [a1, a2, alpha1, alpha2]
    % a1, a2       = Initial angular positions (radians) from vertical
    % alpha1, alpha2 = Initial angular velocities (rad/s)
    a1_0     = pi/2;   % 90 degrees (horizontal)
    a2_0     = pi/2;   % 90 degrees (horizontal)
    alpha1_0 = 0.0;    % Stationary at release
    alpha2_0 = 0.0;    
    
    initial_state = [a1_0; a2_0; alpha1_0; alpha2_0];

    %% --- TIME PARAMETERS ---
    t_max = 10.0;       % Duration of simulation (seconds)
    tspan = 0:0.02:t_max;

    %% --- SOLVE SYSTEM USING NUMERICAL ODE ENGINE ---
    % Pass state derivative rules to ode45
    [t, state_out] = ode45(@(t, y) double_pendulum_dynamics(t, y, params), tspan, initial_state);

    % Extract solution histories
    a1_hist     = state_out(:, 1);
    a2_hist     = state_out(:, 2);
    alpha1_hist = state_out(:, 3);
    alpha2_hist = state_out(:, 4);

    %% --- COMPUTE KINEMATICS FOR PLOTTING ---
    % Convert angular parameters to Cartesian space coordinates
    % Note: Flipped vertical signs relative to notes to hang downward naturally (-y)
    x1 =  params.l1 * sin(a1_hist);
    y1 = -params.l1 * cos(a1_hist);
    
    x2 = x1 + params.l2 * sin(a2_hist);
    y2 = y1 - params.l2 * cos(a2_hist);

    %% --- SETUP ANIMATION GRAPHICS ---
    fig = figure('Name', 'ME 104 Lec 40: Double Pendulum', 'Color', 'w', 'Position', [100, 100, 700, 600]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    % Secure stable tracking frame window
    max_len = params.l1 + params.l2;
    xlim(ax, [-max_len - 0.5, max_len + 0.5]);
    ylim(ax, [-max_len - 0.5, max_len + 0.5]);
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');

    % Graphical object handles
    h_trace = plot(ax, NaN, NaN, 'r-', 'LineWidth', 1); % Tracks path of lower mass
    h_rods  = plot(ax, [0, 0, 0], [0, 0, 0], 'k-o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'b');

    %% --- ANIMATION LOOP ---
    for i = 1:length(t)
        % Update historical trace plot of Mass 2
        set(h_trace, 'XData', x2(1:i), 'YData', y2(1:i));
        
        % Update link lines: [Origin -> Mass 1 -> Mass 2]
        set(h_rods, 'XData', [0, x1(i), x2(i)], 'YData', [0, y1(i), y2(i)]);
        
        title(ax, sprintf('Time: %.2f s | a_1: %.1f^o | a_2: %.1f^o', t(i), rad2deg(a1_hist(i)), rad2deg(a2_hist(i))));
        drawnow;
        pause(0.01);
    end
end

%% ====================================================================
%% State-Space Dynamics Engine: Implements Multi-Parameter Euler-Lagrange Equations
%% ====================================================================
function dstate = double_pendulum_dynamics(~, state, params)
    % Extract state elements
    a1     = state(1);
    a2     = state(2);
    alpha1 = state(3);
    alpha2 = state(4);

    % Unpack design parameters
    m1 = params.m1; m2 = params.m2;
    l1 = params.l1; l2 = params.l2;
    g  = params.g;

    %% Matrix System Assembly from Equations of Motion %%
    % Hand-evaluating the analytical partial derivative system from page 7:
    % M(a) * [alpha1_dot; alpha2_dot] = F(a, alpha)
    
    % Mass Matrix Coefficients (Inertial properties)
    M11 = (m1 + m2) * l1^2;
    M12 = m2 * l1 * l2 * cos(a1 - a2);
    M21 = m2 * l1 * l2 * cos(a1 - a2);
    M22 = m2 * l2^2;
    
    M = [M11, M12; 
         M21, M22];

    % Forcing Vector Coefficients (Coriolis, Centrifugal, and Gravitational Torques)
    F1 = -m2 * l1 * l2 * alpha2^2 * sin(a1 - a2) - (m1 + m2) * g * l1 * sin(a1);
    F2 =  m2 * l1 * l2 * alpha1^2 * sin(a1 - a2) - m2 * g * l2 * sin(a2);
    
    F = [F1; 
         F2];

    %% Solve for Accelerations via Linear Algebra %%
    % [alpha1_dot; alpha2_dot] = M \ F
    alphadot = M \ F;

    % Pack derivatives back up into state format: [a1_dot; a2_dot; alpha1_dot; alpha2_dot]
    dstate = [alpha1; 
              alpha2; 
              alphadot(1); 
              alphadot(2)];
end
