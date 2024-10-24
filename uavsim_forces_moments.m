% uavsim_forces_moments.m
%
% Generation of forces and moments acting on vehicle for uavsim
%
% Inputs:
%   Wind in NED frame
%   Control surfaces
%   UAV States
%   Time
%
% Outputs:
%   Forces in body frame
%   Moments in body frame
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = uavsim_forces_moments(uu, P)

    % Extract variables from input vector uu
    %   uu = [wind_ned(1:3); deltas(1:4); x(1:12); time(1)];
    k=(1:3);          wind_ned=uu(k);   % Total wind vector, ned, m/s
    k=k(end)+(1:4);   deltas=uu(k);     % Control surface commands: [delta_e delta_a delta_r delta_t]
    k=k(end)+(1:12);  x=uu(k);          % states
    k=k(end)+(1);     time=uu(k);       % Simulation time, s

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

    % Combine states to vector form for convenience
    P_ned = [pn; pe; pd];   % NED position, m
    vg_b  = [u; v; w];      % Groundspeed vector, body frame, m/s
    w_b   = [p; q; r];      % body rates about x,y,z, rad/s

    % Extract control commands from deltas
    delta_e = deltas(1); % Elevator, radians
    delta_a = deltas(2); % Aileron, radians
    delta_r = deltas(3); % Rudder, radians
    delta_t = deltas(4); % Throttle, 0-1

    % Your code goes below...
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);

    % compute wind vector in body frame (wind_ned is an input)
    wind_b = R_ned2b*wind_ned;
    v_rel_b = vg_b - wind_b;

    % compute airspeed Va, angle-of-attack alpha, side-slip 
  
    [Va, alpha, beta] = makeVaAlphaBeta(v_rel_b);

    % Longitudinal Aero Coefficients
    C_L = P.C_L_0 + P.C_L_alpha*alpha + P.C_L_q*((P.c*q)/(2*Va)) +P.C_L_delta_e*delta_e;
    C_D = P.C_D_0 + abs(P.C_D_alpha*alpha) + abs(P.C_D_q*((P.c*q)/(2*Va))) + abs(P.C_D_delta_e*delta_e);
    C_m = P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*((P.c*q)/(2*Va)) + P.C_m_delta_e*delta_e;

    % Lateral Aero Coefficients
    C_Y = P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*((P.b*p)/(2*Va)) + P.C_Y_r*((P.b*r)/(2*Va))  + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;
    C_ell = P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*((P.b*p)/(2*Va)) + P.C_ell_r*((P.b*r)/(2*Va))  + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r;
    C_n   = P.C_n_0 + P.C_n_beta*beta + P.C_n_p*((P.b*p)/(2*Va)) + P.C_n_r*((P.b*r)/(2*Va))  + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r;

    % Create and combine Forces    
    f_grav_b = P.mass*P.gravity*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];
    f_aero_b = 0.5*P.rho*Va^2*P.S_wing*[-C_D*cos(alpha) + C_L*sin(alpha); C_Y; -C_D*sin(alpha) - C_L*cos(alpha)];
    f_prop_b =  P.rho*P.C_prop*P.S_prop*(Va+delta_t*(P.k_motor-Va))*delta_t*(P.k_motor-Va)*[1;0;0];
    f_b = f_grav_b + f_aero_b + f_prop_b;

    % Create and combine Moments    
    m_aero_b = 0.5*P.rho*Va^2*P.S_wing*[P.b*C_ell; P.c*C_m; P.b*C_n];
    m_prop_b = -P.k_Tp*(P.k_omega*delta_t)^2*[1;0;0];
    m_b = m_aero_b + m_prop_b;

    % Compile function output
    out = [f_b; m_b]; % Length 3+3=6
    
end