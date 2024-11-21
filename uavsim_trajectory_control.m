function out = uavsim_trajectory_control(uu)
    k=(1:23); estimates=uu(k); % Feedback state estimates
    time=uu(end); 
    
    % Waypoints WP1 WP2 WP3 WP4 WP5
    wp_east = [ 0 400 500 500 0]; % m
    wp_north = [100 100 300 500 300]; % m
    wp_alt = [ 50 60 90 90 60]; % m
    wp_speed = [ 13 13 13 16 16]; % m/s

    % Extract variables from estimates
    pn_hat       = estimates(1);  % inertial North position, m
    pe_hat       = estimates(2);  % inertial East position, m
    h_hat        = estimates(3);  % altitude, m
    Va_hat       = estimates(4);  % airspeed, m/s
    phi_hat      = estimates(5);  % roll angle, rad
    theta_hat    = estimates(6);  % pitch angle, rad
    psi_hat      = estimates(7);  % yaw angle, rad
    p_hat        = estimates(8);  % body frame roll rate, rad/s
    q_hat        = estimates(9);  % body frame pitch rate, rad/s
    r_hat        = estimates(10); % body frame yaw rate, rad/s
    Vn_hat       = estimates(11); % north speed, m/s
    Ve_hat       = estimates(12); % east speed, m/s
    Vd_hat       = estimates(13); % downward speed, m/s
    wn_hat       = estimates(14); % wind North, m/s
    we_hat       = estimates(15); % wind East, m/s  
    chi_hat = atan2(Ve_hat,Vn_hat);
    future_use   = estimates(16:23);
    
    persistent kWP
    if(time==0)
        kWP = 1;
    end

    % Extract variables from traj_cmds
    dist_to_wp = sqrt((wp_east(kWP)-pe_hat)^2 + (wp_north(kWP)-pn_hat)^2);
    dist_to_wp_threshold=20;

    if( dist_to_wp < dist_to_wp_threshold )
        kWP=kWP+1;
        if kWP>length(wp_east)
            kWP=1;
        end
    end

    % Set waypoint leg commands
    chi_c = atan2(wp_east(kWP)-pe_hat,wp_north(kWP)-pn_hat);
    h_c = wp_alt(kWP);
    Va_c = wp_speed(kWP);
    out=[Va_c;h_c;chi_c];
end