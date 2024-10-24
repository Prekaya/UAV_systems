% uavsim_sensors.m
%
% Generation of sensor measurements for uavsim
%
% Inputs:
%   Forces and Moments (used to create accelerometer measurement)
%   UAV States
%   Wind vector
%   Time
%
% Outputs:
%   Sensor Measurements
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = uavsim_sensors(uu, P)

    % Extract variables from input vector uu
    %   uu = [f_and_m(1:6); x(1:12); wind_ned(1:3); time(1)];
    k=(1:6);           f_and_m=uu(k);   % Forces and Moments, body
    k=k(end)+(1:12);   x=uu(k);         % states
    k=k(end)+(1:3);    wind_ned=uu(k);  % wind vector, ned, m/s
    k=k(end)+(1);      time=uu(k);      % Simulation time, s

    % Extract forces and moments from f_and_m
    fb_x = f_and_m(1); % Total force along body x, N
    fb_y = f_and_m(2); % Total force along body y, N
    fb_z = f_and_m(3); % Total force along body z, N
    mb_x = f_and_m(4); % Total moment about body x, N-m
    mb_y = f_and_m(5); % Total moment about body y, N-m
    mb_z = f_and_m(6); % Total moment about body z, N-m

    % Extract state variables from x
    pn    = x(1);   % North position, m
    pe    = x(2);   % East position, m
    pd    = x(3);   % Down position, m
    u     = x(4);   % body-x groundspeed component, m/s
    v     = x(5);   % body-y groundspeed component, m/s
    w     = x(6);   % body-z groundspeed component, m/s
    phi   = x(7);   % EulerAngle: roll, rad
    theta = x(8);   % EulerAngle: pitch, rad
    psi   = x(9);   % EulerAngle: yaw, rad
    p     = x(10);  % body rate about x, rad/s
    q     = x(11);  % body rate about y, rad/s
    r     = x(12);  % body rate about z, rad/s

    % Gyro Measurements
    p_gyro = 0; % rad/s
    q_gyro = 0; % rad/s
    r_gyro = 0; % rad/s

    % Accelerometer Measurements
    ax_accel= 0; % m/s^2
    ay_accel= 0; % m/s^2
    az_accel= 0; % m/s^2

    % Barometric Pressure Altimeter (Note: don't overwrite P structure!)
    P0 = 101325;  % Standard pressure at sea level, N/m^2
    R = 8.31432;  % Universal gas constant for air, N-m/(mol-K)
    M = 0.0289644;% Standard molar mass of atmospheric air, kg/mol
    T = 5/9*(P.air_temp_F-32)+273.15; % Air temperature in Kelvin
    persistent bias_static_press
    if(time==0)
        bias_static_press = P.sigma_bias_static_press*randn;
    end
    true_static_press = 0; % True static pressure at UAV altitude (above sea level), N/m^2
    static_press = 0; % Measured static pressure, N/m^2

    % Airspeed Pitot Measurment for axially mounted pitot tube
    persistent bias_diff_press
    if(time==0)
        bias_diff_press = P.sigma_bias_diff_press*randn;
    end
    true_diff_press = 0; % True differential pressure at UAV airspeed
    diff_press = 0; % Measured differential pressure, N/m^2

    % Magnetometer Measurement
    persistent bias_mag
    if(time==0)
        bias_mag = P.sigma_bias_mag*randn;
    end
    psi_mag= 0; % Magnetometer measurement, rad

    % GPS Position and Velocity Measurements
    persistent time_gps_prev ...
               gps_north_error gps_east_error gps_alt_error ...
               pn_gps pe_gps alt_gps Vn_gps Ve_gps Vd_gps
    if(time==0)
        gps_north_error = 0;
        gps_east_error = 0;
        gps_alt_error = 0;
        time_gps_prev = -inf; % Force update at time==0
    end
    if(time>time_gps_prev+P.Ts_gps)
        
        % Gauss-Markov growth of GPS position errors
        gps_north_error = 0;
        gps_east_error  = 0;
        gps_alt_error   = 0;

        % GPS Position Measurements
        pn_gps = 0;
        pe_gps = 0;
        alt_gps= 0;

        % GPS Velocity Measurements
        Vn_gps = 0;
        Ve_gps = 0;
        Vd_gps = 0;

        time_gps_prev = time;
    end

    % Compile output vector
    out = [ ...
            pn_gps; ...
            pe_gps; ...
            alt_gps;...
            Vn_gps; ...
            Ve_gps; ...
            Vd_gps; ...
            p_gyro; ...
            q_gyro; ...
            r_gyro; ...
            ax_accel;...
            ay_accel;...
            az_accel;...
            static_press; ...
            diff_press; ...
            psi_mag;...        
            0; % future use
            0; % future use
            0; % future use
          ]; % Length: 18

end