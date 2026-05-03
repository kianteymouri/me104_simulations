%% 3d animation (vibe coded)
%%warning: prob will crash ur computer!!!!

function conical_surface_movie
    % Parameters [cite: 292, 298]
    m = 1.0;        % Mass (kg)
    g = 9.81;       % Gravity (m/s^2)
    Omega = 5.0;    % Spin rate of cone (rad/s)
    beta = deg2rad(30); % Cone angle
    mu_d = 0.3;     % Kinetic friction coefficient
    
    % Initial Conditions: [r, r_dot, theta, theta_dot]
    % Starting 2 meters up the cone with a small initial velocity
    z0 = [2.0, 0.1, 0, 0.5]; 
    tspan = [0 5];
    
    % Solve Numerically 
    options = odeset('RelTol', 1e-6);
    [t, z] = ode45(@(t, z) equations(t, z, m, g, Omega, beta, mu_d), tspan, z0, options);

    % Animation Setup
    figure('Color', 'w');
    hold on; grid on; axis equal;
    view(3); % 3D Perspective
    xlabel('x'); ylabel('y'); zlabel('z');
    title('Mass on a Rotating Conical Surface');
    
    % Draw the Cone for visual reference
    [cx, cy] = meshgrid(linspace(-5, 5, 20));
    cr = sqrt(cx.^2 + cy.^2);
    cz = cr * tan(beta);
    mesh(cx, cy, cz, 'EdgeColor', [0.8 0.8 0.8], 'FaceAlpha', 0.1);

    % Movie Objects
    path = plot3(nan, nan, nan, 'r', 'LineWidth', 1.5);
    ball = plot3(nan, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    
    % Fixed Axis limits
    limit = max(z(:,1)) * 1.2;
    axis([-limit limit -limit limit 0 limit*tan(beta)]);

    % Animation Loop
    for i = 1:length(t)
        % Convert cylindrical to cartesian for plotting [cite: 309]
        r = z(i, 1);
        theta = z(i, 3);
        x = r * cos(theta);
        y = r * sin(theta);
        curr_z = r * tan(beta);
        
        % Update path and ball
        set(path, 'XData', z(1:i, 1).*cos(z(1:i, 3)), ...
                  'YData', z(1:i, 1).*sin(z(1:i, 3)), ...
                  'ZData', z(1:i, 1).*tan(beta));
        set(ball, 'XData', x, 'YData', y, 'ZData', curr_z);
        
        drawnow;
        pause(0.01);
    end
end

function dzdt = equations(t, z, m, g, Omega, beta, mu_d)
    r = z(1);
    rdot = z(2);
    theta = z(3);
    thetadot = z(4);
    
    % Velocity magnitude in rotating frame [cite: 310, 318]
    v_mag = sqrt(rdot^2 + (r*thetadot)^2 + (rdot*tan(beta))^2);
    if v_mag < 1e-6, v_mag = 1e-6; end % Prevent division by zero
    
    % Solve for Normal Force Fn using the e_z and e_r equations [cite: 325]
    % From e_z: Fn*cos(beta) - mg - F_fric_z = m*z_ddot
    % Since z = r*tan(beta), z_ddot = r_ddot*tan(beta)
    % This requires simultaneous solving for r_ddot and Fn.
    
    % Simplified Fn derived from constraints [cite: 330]
    num = m * (g + r * (thetadot^2 * tan(beta)));
    den = cos(beta) - mu_d * (rdot*tan(beta)/v_mag) - tan(beta)*sin(beta);
    Fn = num / den;
    if Fn < 0, Fn = 0; end % Mass leaves surface

    % r_double_dot (Radial acceleration) [cite: 326]
    % -Fn*sin(beta) + F_fric_r + F_cent + F_cor_r = m(r_ddot - r*theta_dot^2)
    friction_r = -mu_d * Fn * (rdot / v_mag);
    F_cent = m * Omega^2 * r; [cite: 321]
    F_cor_r = 2 * m * Omega * r * thetadot; [cite: 322]
    
    r_ddot = (-Fn*sin(beta) + friction_r + F_cent + F_cor_r) / m + r*thetadot^2;
    
    % theta_double_dot (Tangential acceleration) [cite: 327]
    % F_fric_theta + F_cor_theta = m(r*theta_ddot + 2*rdot*theta_dot)
    friction_theta = -mu_d * Fn * (r*thetadot / v_mag);
    F_cor_theta = -2 * m * Omega * rdot; [cite: 322]
    
    theta_ddot = (friction_theta + F_cor_theta - 2*m*rdot*thetadot) / r;
    
    dzdt = [rdot; r_ddot; thetadot; theta_ddot];
end
