function double_pendulum_simulation()
    %Lecture 40
    % Derived using euler lagrange equations
    clear all; close all; clc;

    %% parameters of system
    params.m1 = 1.0;   % Mass of ball 1 (kg)
    params.m2 = 1.0;   % Mass of ball 2 (kg)
    params.l1 = 1.5;   % Length of rod 1 (m)
    params.l2 = 1.2;   % Length of rod 2 (m)
    params.g  = 9.81;  % Acceleration due to gravity (m/s^2)

    %initial conditions
    % a1, a2 are the angular radians
    % alpha1, alpha2 are the angular velocities
    a1_0     = pi/2; 
    a2_0     = pi/2;  
    alpha1_0 = 0.0;   
    alpha2_0 = 0.0;    
    
    initial_state = [a1_0; a2_0; alpha1_0; alpha2_0];

    %%time param
    t_max = 10.0;       % Duration of simulation (seconds)
    tspan = 0:0.02:t_max;

    %% solving system
    [t, state_out] = ode45(@(t, y) double_pendulum_dynamics(t, y, params), tspan, initial_state);

    %get solution histories
    a1_hist     = state_out(:, 1);
    a2_hist     = state_out(:, 2);
    alpha1_hist = state_out(:, 3);
    alpha2_hist = state_out(:, 4);

    %%kinematics for plotting
    % Note: Flipped vertical signs relative to notes to hang downward naturally (-y)
    x1 =  params.l1 * sin(a1_hist);
    y1 = -params.l1 * cos(a1_hist);
    
    x2 = x1 + params.l2 * sin(a2_hist);
    y2 = y1 - params.l2 * cos(a2_hist);

    %%animation graphics
    fig = figure('Name', 'ME 104 Lec 40: Double Pendulum', 'Color', 'w', 'Position', [100, 100, 700, 600]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    max_len = params.l1 + params.l2;
    xlim(ax, [-max_len - 0.5, max_len + 0.5]);
    ylim(ax, [-max_len - 0.5, max_len + 0.5]);
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');

    h_trace = plot(ax, NaN, NaN, 'r-', 'LineWidth', 1); % Tracks path of lower mass
    h_rods  = plot(ax, [0, 0, 0], [0, 0, 0], 'k-o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerFaceColor', 'b');

    %%movie magic
    for i = 1:length(t)
        %updating trace plot
        set(h_trace, 'XData', x2(1:i), 'YData', y2(1:i));
        
        %updating link lines
        set(h_rods, 'XData', [0, x1(i), x2(i)], 'YData', [0, y1(i), y2(i)]);
        
        title(ax, sprintf('Time: %.2f s | a_1: %.1f^o | a_2: %.1f^o', t(i), rad2deg(a1_hist(i)), rad2deg(a2_hist(i))));
        drawnow;
        pause(0.01);
    end
end

%%state space dynamics
function dstate = double_pendulum_dynamics(~, state, params)
    a1     = state(1);
    a2     = state(2);
    alpha1 = state(3);
    alpha2 = state(4);

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

    %%solve for acclerations
    % [alpha1_dot; alpha2_dot] = M \ F
    alphadot = M \ F;

    %packing derivatives into state format
    dstate = [alpha1; 
              alpha2; 
              alphadot(1); 
              alphadot(2)];
end
