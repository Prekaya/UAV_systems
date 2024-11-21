% uavsim_control.m
%
% Flight control logic for uavsim
%
% Inputs:
%   Trajectory commands
%   State Feedbacks
%   Time
%
% Outputs:
%   Control surface commands
%   Autopilot state commands (for logging and plotting)
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = uavsim_control(uu,P)

    % Extract variables from input vector uu
    %   uu = [traj_cmds(1:3); estimates(1:23); time(1)];
    k=(1:3);         traj_cmds=uu(k); % Trajectory Commands
    k=k(end)+(1:23); estimates=uu(k); % Feedback state estimates
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

    % Extract variables from traj_cmds
    Va_c     = traj_cmds(1);  % commanded airspeed (m/s)
    h_c      = traj_cmds(2);  % commanded altitude (m)
    chi_c    = traj_cmds(3);  % commanded course (rad)

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

    
    % Initialize controls to trim (to be set with autopilot logic)
    delta_e=P.delta_e0;
    delta_a=P.delta_a0;
    delta_r=P.delta_r0;
    delta_t=P.delta_t0;
    
    %% Set "first-time" flag, which is used to initialize autopilot integrators
    firstTime=(time==0);

    %% Initialize autopilot commands (may be overwritten with autopilot logic)
    % P.altitude_kp = 0.06; % kp>0
    % P.altitude_ki = 0.02; % ki>0
    % P.altitude_kd = 0; % <-- Don’t use
    % if time<10,
    %     h_c=50; Va_c=13; chi_c=0*pi/180;
    % elseif time<20
    %     h_c=50; Va_c=13; chi_c=60*pi/180;
    % else
    %     h_c=100; Va_c=15; chi_c=0*pi/180;
    % end
    %% Flight control logic
    if(firstTime)
    % Initialize integrators
        PIR_pitch_hold(0,0,0,firstTime,P);
        PIR_alt_hold_using_pitch(0,0,0,firstTime,P);
        PIR_airspeed_hold_using_throttle(0,0,0,firstTime,P);
        PIR_airspeed_hold_using_pitch(0,0,0,firstTime,P);
    end
    h_hold = 5; % m, alt threshold
    if h_hat < h_c - h_hold % Climbing Mode
        theta_c = PIR_airspeed_hold_using_pitch(Va_c, Va_hat, 0, firstTime, P);
        delta_e = PIR_pitch_hold(theta_c, theta_hat, q_hat, firstTime, P);
        delta_t = 1;
    elseif h_hat > h_c + h_hold % Descending Mode
        theta_c = PIR_airspeed_hold_using_pitch(Va_c, Va_hat, 0, firstTime, P);
        delta_e = PIR_pitch_hold(theta_c, theta_hat, q_hat, firstTime, P);
        delta_t = 0;
    else % Altitude Hold
        theta_c = PIR_alt_hold_using_pitch(h_c, h_hat, 0, firstTime, P);
        delta_e = PIR_pitch_hold(theta_c, theta_hat, q_hat, firstTime, P);
        delta_t = PIR_airspeed_hold_using_throttle(Va_c, Va_hat, 0, firstTime, P);
    end
    phi_c = PIR_course_hold(chi_c, chi_hat, r_hat, firstTime, P);
    delta_a = PIR_roll_hold(phi_c, phi_hat, p_hat, firstTime, P);

    % Compile vector of control surface deflections
    delta = [ ...
            delta_e; ...
            delta_a; ...
            delta_r; ...
            delta_t; ...
        ];

    % Override control delta with manual flight delta
    if P.manual_flight_flag
        delta = uavsim_manual_control(time,P);
    end

    % Compile autopilot commands for logging/vis
    ap_command = [ ...
            Va_c; ...
            h_c; ...
            chi_c; ...
            phi_c; ...
            theta_c; 
            0; ... % future use
            0; ... % future use
            0; ... % future use
            0; ... % future use
        ];

    % Compile output vector
    out=[delta; ap_command]; % 4+9=13

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%
%   Autopilot controllers in UAVSIM are based on "PI with rate feedback".
%   For convenience, we'll refer to "PI with rate feedback" as "PIR".
%
%   u = PIR_xxx(y_c, y, y_dot, init_flag, dt)
%     Inputs:
%       y_c:    Closed loop command
%       y:      Current system response
%       y_dot:  Rate feedback (derivative of y)
%       init_flag:  1: initialize integrator, 0: otherwise
%       dt:     Time step, seconds
%     Outputs:
%       u:      Controller output (input to Gplant)
%                         
%                .------.           .---- Limit on plant input
%             .->| ki/s |---.       |     (a.k.a. limit on controller output)
%             |  '------'   |+      v
%   Input     |  .------. + v  +    -. u  .------. .---.     .---.  Output
%    ---->( )-'->|  kp  |->( )->( )--|--->|Gplant|-| s |--.--|1/s|--.--->
%   y_c    ^     '------'        ^  -'    '------' '---'  |  '---'  |  y
%         -|                    -|         .------.       |         |
%          |                     '---------|  kd  |<------'y_dot    |
%          |                               '------'                 |
%          '--------------------------------------------------------'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_pitch_hold(theta_c, theta_hat, q_hat, init_flag, P)
   

    % Set up PI with rate feedback
    y_c = theta_c; % Command
    y = theta_hat; % Feedback
    y_dot = q_hat; % Rate feedback
    kp = P.pitch_kp;
    ki = P.pitch_ki;
    kd = P.pitch_kd;
    u_lower_limit = -P.delta_e_max;
    u_upper_limit = +P.delta_e_max;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% alt_hold
%   - regulate altitude using pitch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_alt_hold_using_pitch(h_c, h_hat, not_used, init_flag, P)

    % Set up PI with rate feedback
    y_c = h_c;   % Command
    y = h_hat;   % Feedback
    y_dot = 0;   % Rate feedback
    kp = P.altitude_kp;
    ki = P.altitude_ki;
    kd = P.altitude_kd;
    u_lower_limit = -P.theta_max/P.K_theta_DC;
    u_upper_limit = +P.theta_max/P.K_theta_DC;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_roll_hold(phi_c, phi_hat, p_hat, init_flag, P)

% Set up PI with rate feedback
    y_c = phi_c; % Command
    y = phi_hat; % Feedback
    y_dot = p_hat; % Rate feedback
    kp = P.roll_kp;
    ki = P.roll_ki;
    kd = P.roll_kd;
    u_lower_limit = -P.delta_a_max;
    u_upper_limit = +P.delta_a_max;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
end

function u = PIR_course_hold(chi_c, chi_hat, r_hat, init_flag, P)

% Set up PI with rate feedback
    y_c = chi_c; % Command
    y = chi_hat; % Feedback
    y_dot = r_hat; % Rate feedback
    kp = P.course_kp;
    ki = P.course_ki;
    kd = P.course_kd;
    u_lower_limit = -P.phi_max;
    u_upper_limit = +P.phi_max;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = mod(y_c-y+pi,2*pi)-pi;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_hold_throttle
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_airspeed_hold_using_throttle(Va_c, Va_hat, not_used, init_flag, P)

% Set up PI with rate feedback
    y_c = Va_c; % Command
    y = Va_hat; % Feedback
    y_dot = 0; % Rate feedback
    kp = P.airspeed_throttle_kp;
    ki = P.airspeed_throttle_ki;
    kd = P.airspeed_throttle_kd;
    u_lower_limit = 0;
    u_upper_limit = 1;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_hold_pitch
%   - regulate airspeed using pitch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = PIR_airspeed_hold_using_pitch(Va_c, Va_hat, not_used, init_flag, P)

% Set up PI with rate feedback
    y_c = Va_c; % Command
    y = Va_hat; % Feedback
    y_dot = 0; % Rate feedback
    kp = P.airspeed_pitch_kp;
    ki = P.airspeed_pitch_kp;
    kd = P.airspeed_pitch_kp;
    u_lower_limit = -P.theta_max/P.K_theta_DC;
    u_upper_limit = +P.theta_max/P.K_theta_DC;

    % Initialize integrator (e.g. when t==0)
    persistent error_int;
    if( init_flag )   
        error_int = 0;
    end  

    % Perform "PI with rate feedback"
    error = y_c - y;  % Error between command and response
    error_int = error_int + P.Ts*error; % Update integrator
    u = kp*error + ki*error_int - kd*y_dot;

    % Output saturation & integrator clamping
    %   - Limit u to u_upper_limit & u_lower_limit
    %   - Clamp if error is driving u past limit
    if u > u_upper_limit
        u = u_upper_limit;
        if ki*error>0
            error_int = error_int - P.Ts*error;
        end
    elseif u < u_lower_limit
        u = u_lower_limit;
        if ki*error<0
            error_int = error_int - P.Ts*error;
        end
    end
end



