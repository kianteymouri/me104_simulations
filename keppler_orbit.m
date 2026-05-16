function orbital_simulation_static()
    % ME 104: Static Orbital Mechanics Simulation
    % Trajectory Equation: r(theta) = p / (1 + e * cos(theta - phi)) [cite: 49]

    %% ====================================================================
    %% STUDENT CONFIGURATION ZONE: Change these variables to alter the orbit
    %% ====================================================================
    e       = 0.7;    % Eccentricity (e = 0: Circle, 0 < e < 1: Ellipse, e = 1: Parabola, e > 1: Hyperbola) [cite: 56, 57, 58, 59]
    phi_deg = 0;     % Offset/Rotation angle in DEGREES [cite: 52]
    p       = 2.0;    % Trajectory parameter (controls the scale of the orbit) [cite: 50, 51]
    %% ====================================================================

    % Convert offset angle to radians for calculation [cite: 52]
    phi = deg2rad(phi_deg); 

    % Generate an evaluation sweep for theta from 0 to 2*pi [cite: 42, 47]
    theta = linspace(0, 2*pi, 1000);
    
    % Avoid division by zero or negative radii for unbound/escape trajectories [cite: 49]
    denominator = 1 + e * cos(theta - phi); 
    valid_idx = denominator > 1e-4; 
    
    r = NaN(size(theta));
    r(valid_idx) = p ./ denominator(valid_idx); 

    % Convert polar coordinates (r, theta) to Cartesian (x, y) relative to e_x and e_y [cite: 4, 5, 35, 38]
    x = r .* cos(theta);
    y = r .* sin(theta);

    %% --- Calculate Metrics and Classify Orbit ---
    if e == 0
        orbit_type = 'Circle'; 
        rp = p;
        ra = p;
    elseif e < 1
        orbit_type = 'Ellipse'; 
        rp = p / (1 + e); % Closest approach [cite: 65, 66]
        ra = p / (1 - e); % Farthest approach [cite: 67, 68]
    elseif e == 1
        orbit_type = 'Parabola'; 
        rp = p / 2; 
        ra = Inf;
    else
        orbit_type = 'Hyperbola'; 
        rp = p / (1 + e); 
        ra = Inf;
    end

    %% --- Command Window Readout ---
    fprintf('\n======================================\n');
    fprintf('  ME 104 ORBITAL TRAJECTORY METRICS\n');
    fprintf('======================================\n');
    fprintf('Trajectory Class : %s\n', orbit_type);
    fprintf('Parameter (p)    : %.2f\n', p); 
    fprintf('Eccentricity (e) : %.2f\n', e); 
    fprintf('Offset Angle     : %.1f°\n', phi_deg); 
    fprintf('Periapsis (r_p)  : %.2f\n', rp); 
    if isfinite(ra)
        fprintf('Apoapsis (r_a)   : %.2f\n', ra);
    else
        fprintf('Apoapsis (r_a)   : Infinity (Escape Path)\n'); 
    end
    fprintf('======================================\n');

    %% --- Plotting ---
    fig = figure('Name', 'ME 104: Static Orbital Trajectory', 'Color', 'w');
    ax = axes('Parent', fig);
    grid(ax, 'on');
    axis(ax, 'equal');
    hold(ax, 'on');
    xlabel(ax, 'x (\underline{e}_x)'); 
    ylabel(ax, 'y (\underline{e}_y)'); 
    title(ax, sprintf('Orbital Shape: %s (e = %.2f)', orbit_type, e));

    % Plot Central Mass M at the origin focus [cite: 8, 61, 64]
    plot(ax, 0, 0, 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Mass M'); 

    % Plot the calculated trajectory path [cite: 11, 46]
    plot(ax, x, y, 'r-', 'LineWidth', 2, 'DisplayName', 'Trajectory');

    % Plot the Periapsis point (magenta dot) [cite: 65]
    xp = rp * cos(phi);
    yp = rp * sin(phi);
    plot(ax, xp, yp, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm', 'DisplayName', 'Periapsis (r_p)');

    % Plot Apoapsis point if it's a closed/bound orbit (cyan dot) [cite: 67]
    if isfinite(ra)
        xa = ra * cos(phi + pi);
        ya = ra * sin(phi + pi);
        plot(ax, xa, ya, 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c', 'DisplayName', 'Apoapsis (r_a)');
    end

    legend(ax, 'show', 'Location', 'best');

    % Dynamically set layout bounds based on the orbit size so it frames beautifully
    padding = rp * 2.5;
    if isfinite(ra) && ra < 25
        padding = ra * 1.2;
    end
    xlim(ax, [-padding, padding]);
    ylim(ax, [-padding, padding]);
end
