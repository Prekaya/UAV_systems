% uavsim_logging.m
%
% Logging of uavsim variables
%
% Inputs:
%   Various 
%
% Outputs:
%   Creates a "out" structure in the Matlab workspace
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function uavsim_logging(uu,P)

    % Logging flags
    log_commands     = 1;
    log_measurements = 0;
    log_estimates    = 0;
    
    % Extract variables from input vector uu
    %   uu = [x(1:); f_and_m(1:6); wind_ned(1:3); ap_cmds(1:9); estimates(1:23); meas(1:18); time(1)];
    k=1:12;          x=uu(k);         % States
    k=k(end)+(1:6);  f_and_m=uu(k);   % Forces and Moments, body
    k=k(end)+(1:3);  wind_ned=uu(k);  % Wind vector, ned, m/s
    k=k(end)+(1:4);  deltas=uu(k);    % Control commands
    k=k(end)+(1:9);  ap_cmds=uu(k);   % Autopilot commands
    k=k(end)+(1:23); estimates=uu(k); % Autopilot state estimates
    k=k(end)+(1:18); meas=uu(k);      % Measurements
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

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

    % Extract control commands
    delta_e   = deltas(1); % Elevator, rad
    delta_a   = deltas(2); % Aileron, rad
    delta_r   = deltas(3); % Rudder, rad
    delta_t   = deltas(4); % Throttle, 0-1

    % Compute Rotation Matrices
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);
    R_b2ned = R_ned2b';

    % Manipulate states
    vg_ned = R_b2ned*[u; v; w];
    [Vg, gamma, course] = makeVgGammaCourse(vg_ned);

    % Extract body-frame forces and moments
    f_b   = f_and_m(1:3); % External forces along body x,y,z, N
    m_b   = f_and_m(4:6); % External moments about body x,y,z, N-m

    % Rotate wind vector to body frame
    wind_b = R_ned2b*wind_ned;

    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    u_rel = u - wind_b(1);
    v_rel = v - wind_b(2);
    w_rel = w - wind_b(3);
    v_rel_b = [u_rel; v_rel; w_rel];
    [Va, alpha, beta] = makeVaAlphaBeta(v_rel_b);

    % Use a persistent variable for incrementing logging index
    % Re-initialize "out" structure if first time through
    persistent i
    if time==0
        i=0;
        out = [];
        assignin('base','out',out);
    end
    i=i+1;

    % Acquire out structure from base workspace
    out = evalin('base','out');

    % Append new data to out structure
    % (Note: This is highly inefficient, but it is convenient)

    out.time_s(i) = time;

    out.north_m(i) = pn;
    out.east_m(i) = pe;
    out.alt_m(i) = -pd;
    out.groundspeed_mps(i) = Vg;
    out.gamma_deg(i) = gamma*180/pi;    
    out.course_deg(i) = mod(course*180/pi+180,360)-180; % Limit: -180 to 180
    out.roll_deg(i) = mod(phi*180/pi+180,360)-180; % Limit: -180 to 180
    out.pitch_deg(i) = theta*180/pi;
    out.yaw_deg(i) = mod(psi*180/pi+180,360)-180; % Limit: -180 to 180
    out.p_dps(i) = p*180/pi;
    out.q_dps(i) = q*180/pi;
    out.r_dps(i) = r*180/pi;
    out.alt_rate_mps(i) = Vg*sin(gamma);
    out.horz_speed_mps(i) = Vg*cos(gamma);    
    
    out.airspeed_mps(i) = Va;
    out.alpha_deg(i) = alpha*180/pi;
    out.beta_deg(i) = beta*180/pi;

    out.wind_north_mps(i) = wind_ned(1);
    out.wind_east_mps(i) = wind_ned(2);
    out.wind_down_mps(i) = wind_ned(3);

    out.de_deg(i) = delta_e*180/pi;
    out.da_deg(i) = delta_a*180/pi;
    out.dr_deg(i) = delta_r*180/pi;
    out.throttle(i) = delta_t;

    out.fx_N(i) = f_b(1);
    out.fy_N(i) = f_b(2);
    out.fz_N(i) = f_b(3);
    out.mx_Nm(i) = m_b(1);
    out.my_Nm(i) = m_b(2);
    out.mz_Nm(i) = m_b(3);

    % Extract a/p commands
    if log_commands
        out.airspeed_cmd_mps(i) = ap_cmds(1);
        out.alt_cmd_m(i)        = ap_cmds(2);
        out.course_cmd_deg(i)   = mod(ap_cmds(3)*180/pi+180,360)-180; % Limit: -180 to 180
        out.roll_cmd_deg(i)     = mod(ap_cmds(4)*180/pi+180,360)-180; % Limit: -180 to 180
        out.pitch_cmd_deg(i)    = ap_cmds(5)*180/pi;
    end
    
    % Extract sensor measurements
    if log_measurements
        out.north_gps_m(i)     = meas(1); 
        out.east_gps_m(i)      = meas(2);
        out.alt_gps_m(i)       = meas(3);
        [Vg_gps gamma_gps course_gps] = makeVgGammaCourse(meas(4:6));
        out.groundspeed_gps_mps(i) = Vg_gps;
        out.gamma_gps_deg(i)   = 180/pi*gamma_gps;
        out.course_gps_deg(i)  = 180/pi*course_gps;
        out.p_gyro_dps(i)      = meas(7)*180/pi;
        out.q_gyro_dps(i)      = meas(8)*180/pi;
        out.r_gyro_dps(i)      = meas(9)*180/pi;
        out.ax_accel_mps2(i)   = meas(10);
        out.ay_accel_mps2(i)   = meas(11);
        out.az_accel_mps2(i)   = meas(12);
        out.static_press_Npm2(i) = meas(13);
        out.diff_press_Npm2(i)   = meas(14);
        out.yaw_mag_deg(i)     = mod(meas(15)*180/pi+180,360)-180; % Limit: -180 to 180
    end

    % Extract state estimates
    if log_estimates
        out.alt_baro_m(i)         = estimates(3);
        out.airspeed_pitot_mps(i) = estimates(4);
        out.roll_est_deg(i) = mod(estimates(5)*180/pi+180,360)-180; % Limit: -180 to 180
        out.pitch_est_deg(i) = estimates(6)*180/pi;
    end
    
    % For profiling purposes
    persistent tNow0
    if time==0, 
        try, tNow0=toc; catch, tic; tNow0=toc; end
    end
    out.real_time_s(i) = toc-tNow0; % elapsed wall clock time, s
    
    % Write out structure back to workspace
    assignin('base','out',out);

    % Apply stopping conditions
    %  - Ground
    %  - Extreme accelerations or body rates (linear aero model isn't valid
    %    at extreme conditions):
    %      - 400 deg/s
    %  - Excessive aero angles (alpha or beta)
    if (-pd < 0) && (gamma<0)
        fprintf('Sim stopped due to ground clobber\n');
        error(['Sim stopped: Altitude below ground (see ' mfilename '.m)'])
    end
    if abs(p)*180/pi>400 || abs(q)*180/pi>400 || abs(r)*180/pi>400
        fprintf('Sim stopped due to excessive body rates:\n');
        fprintf('    p = %.1f deg\n',p*180/pi)
        fprintf('    q = %.1f deg\n',q*180/pi)
        fprintf('    r = %.1f deg\n',r*180/pi)
        error(['Sim stopped: Excessive body rate (see ' mfilename '.m)'])
    end
    if (abs(alpha)>45*pi/180 || abs(beta)>45*pi/180) && any(m_b~=0) && any(f_b~=0)
        % Only stop due to alpha or beta if actually simulation full
        % aerodyanmic 6dof with forces and moments 
        % (e.g. Don't stop based on alpha when simulating ballistic trajectory) 
        error(['Sim stopped: Aero data is not valid for high alpha & beta (see ' mfilename '.m)'])
    end
    
    
end