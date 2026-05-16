function rolling_wheels_rigid_bar_universal()
    % ME 104 Lecture 28: Connected Rolling Wheels (Strictly Rigid Bar Enforced)
    % Universal version: Runs on base MATLAB without requiring Optimization Toolbox.
    clear all; close all; clc;

    %% --- PARAMETER CONFIGURATION ZONE ---
    R1 = 1.0;        % Radius of wheel 1
    R2 = 1.5;        % Radius of wheel 2
    D0 = 4.0;        % Initial horizontal distance between wheel centers
    
    % Input kinematics for Wheel 1 (The Driver)
    omega1_init = 1.5;  % Initial angular velocity of wheel 1 (rad/s)
    alpha1      = 0.2;  % Constant angular acceleration of wheel 1 (rad/s^2)

    %% --- TIME PARAMETERS ---
    dt = 0.01;          
    t_max = 5.0;        
    time = 0:dt:t_max;
    
    %% --- INITIALIZE GEOMETRY ---
    % True rigid bar length calculated from the exact initial snapshot in notes
    % Pin A is R1/2 above Center 1, Pin B is R2/2 above Center 2
    L_AB = sqrt(D0^2 + ( (R2 + R2/2) - (R1 + R1/2) )^2);
    
    % Track positions over time
    xc1 = 0;            % Center of wheel 1 moves based on pure rolling
    theta1 = 0;         % Angle of wheel 1
    omega1 = omega1_init;

    %% --- SETUP GRAPHICS WINDOW ---
    fig = figure('Name', 'ME 104: Perfectly Rigid Bar Simulation', 'Color', 'w', 'Position', [100, 100, 900, 450]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    % FIXED: Single backslash and explicit Latex interpreter for formatting
    xlabel(ax, 'x (\underline{e}_x)', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel(ax, 'y (\underline{e}_y)', 'Interpreter', 'latex', 'FontSize', 12);

    %% --- ANIMATION LOOP ---
    for t = time
        %% 1. UPDATE DRIVER (WHEEL 1) GEOMETRY %%
        vc1 = -omega1 * R1;           % Velocity of center 1
        xc1 = xc1 + vc1 * dt;         % Move center 1
        theta1 = theta1 + omega1 * dt; % Rotate wheel 1
        
        C1 = [xc1, R1];
        % Instantaneous position of Pin A (riding on Wheel 1)
        A = C1 + [-(R1/2)*sin(theta1), (R1/2)*cos(theta1)];

        %% 2. ENFORCE RIGID LINKAGE KINEMATICS FOR WHEEL 2 %%
        % FIXED: Replaced fsolve with fzero (native MATLAB) to find theta2
        % Guess close to the expected rolling angle
        theta2_guess = theta1 * (R1/R2); 
        
        % Use fzero to find the exact theta2 that drives the distance error to 0
        options = optimset('Display', 'off', 'TolX', 1e-6);
        theta2 = fzero(@(t2) distance_error_root(t2, A, xc1, D0, R2, L_AB), theta2_guess, options);
        
        % Reconstruct Wheel 2 center via pure rolling constraint relative to theta2
        xc2 = D0 - theta2 * R2; 
        C2 = [xc2, R2];
        B = C2 + [-(R2/2)*sin(theta2), (R2/2)*cos(theta2)];

        %% 3. CALCULATE METRICS TO SHOW STUDENTS %%
        current_bar_length = norm(B - A);

        %% 4. DRAW SYSTEM GRAPHICS %%
        cla(ax); 
        plot(ax, [-20, 25], [0, 0], 'k-', 'LineWidth', 2); % Floor
        
        % Draw Wheels
        draw_wheel(ax, C1, R1, theta1, [0.2 0.6 0.8]);
        draw_wheel(ax, C2, R2, theta2, [0.8 0.4 0.2]);
        
        % Draw Perfectly Rigid Bar AB
        plot(ax, [A(1), B(1)], [A(2), B(2)], 'g-', 'LineWidth', 4);
        
        % Draw Pins
        plot(ax, A(1), A(2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        text(ax, A(1)-0.2, A(2)+0.3, 'A', 'FontWeight', 'bold');
        plot(ax, B(1), B(2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
        text(ax, B(1)-0.2, B(2)+0.3, 'B', 'FontWeight', 'bold');

        % Center camera dynamically
        xlim(ax, [xc1 - 3*R1, xc2 + 3*R2]);
        ylim(ax, [-0.5, max(R1, R2) * 2.8]);
        
        title_str = sprintf('Time: %.2fs | Bar Length: %.4f (Constant!)', t, current_bar_length);
        title(ax, title_str, 'FontSize', 11);
        
        drawnow;
        
        % Advance driver state
        omega1 = omega1 + alpha1 * dt; 
    end
end

%% ====================================================================
%% Root Function: Computes distance error for a given Wheel 2 angle
%% ====================================================================
function error_val = distance_error_root(theta2, A, xc1, D0, R2, L_AB)
    % Compute xc2 assuming pure rolling constraint holds true
    xc2 = D0 - theta2 * R2;
    
    % Calculate Pin B position based on this trial configuration
    B = [xc2, R2] + [-(R2/2)*sin(theta2), (R2/2)*cos(theta2)];
    
    % Return difference from target rigid length
    error_val = norm(B - A) - L_AB;
end

%% ====================================================================
%% Helper Function: Render Circle and Spinning Spoke Elements
%% ====================================================================
function draw_wheel(ax, center, radius, angle, color)
    theta_circle = linspace(0, 2*pi, 100);
    x_rim = center(1) + radius * cos(theta_circle);
    y_rim = center(2) + radius * sin(theta_circle);
    plot(ax, x_rim, y_rim, 'Color', color, 'LineWidth', 2.5);
    
    num_spokes = 4;
    for i = 1:num_spokes
        spoke_angle = angle + (i - 1) * (pi / 2) + pi/2;
        x_spoke = center(1) + radius * cos(spoke_angle);
        y_spoke = center(2) + radius * sin(spoke_angle);
        plot(ax, [center(1), x_spoke], [center(2), y_spoke], 'k-', 'LineWidth', 1);
    end
    plot(ax, center(1), center(2), 'k.', 'MarkerSize', 10);
end
