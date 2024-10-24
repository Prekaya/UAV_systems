% These are the properties as included in book website project, Feb 2014
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   

% physical parameters of airframe
P.mass = 1.56;    % kg
P.Jx   = 0.1147;  % kg-m^2
P.Jy   = 0.0576;  % kg-m^2
P.Jz   = 0.1712;  % kg-m^2
P.Jxz  = 0.0015;  % kg-m^2

% aerodynamic coefficients
P.c             = 0.3302;   % Wing chord, m
P.b             = 1.4224;   % Wing span, m
P.S_wing        = 0.2589;   % Wing area, m^2
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;     % 1/rad
P.C_L_q         = 0.0;      % 1/rad
P.C_L_delta_e   = 0.36;     % 1/rad [Note: Book website had -0.36. Corrected value is +0.36]
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.2108;   % 1/rad
P.C_D_q         = 0.0;      % 1/rad
P.C_D_delta_e   = 0.0;      % 1/rad
P.C_m_0         = 0.0;
P.C_m_alpha     = -0.38;    % 1/rad
P.C_m_q         = -3.6;     % 1/rad
P.C_m_delta_e   = -0.5;     % 1/rad
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;    % 1/rad
P.C_Y_p         = -0.26;    % 1/rad
P.C_Y_r         = 0.0;      % 1/rad
P.C_Y_delta_a   = 0.0;      % 1/rad
P.C_Y_delta_r   = -0.17;    % 1/rad
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;    % 1/rad
P.C_ell_p       = -0.26;    % 1/rad
P.C_ell_r       = 0.14;     % 1/rad
P.C_ell_delta_a = 0.08;     % 1/rad
P.C_ell_delta_r = 0.105;    % 1/rad
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;     % 1/rad
P.C_n_p         = 0.022;    % 1/rad
P.C_n_r         = -0.35;    % 1/rad
P.C_n_delta_a   = 0.06;     % 1/rad
P.C_n_delta_r   = -0.032;   % 1/rad

% Prop params
P.k_motor       = 20;           % Motor constant, m/s
P.S_prop        = 0.0314;       % Aero swept by prop, m^2 (Approx. 8" diameter prop)
P.C_prop        = 1;            % Prop efficientcy coefficient, no units
P.k_omega       = 6000*2*pi/60; % Prop speed constant, rad/s (6000 RPM)
P.k_Tp          = 0;            % Prop torque constant, kg-m^2

