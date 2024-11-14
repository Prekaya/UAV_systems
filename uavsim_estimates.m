% uavsim_estimates.m
%
% Generation of feedback state estimates for uavsim
%
% Inputs:
%   Measurements
%   Time
%
% Outputs:
%   Feedback state estimates
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = uavsim_estimates(uu,P)

    % Extract variables from input vector uu
    %   uu = [meas(1:18); time(1)];
    k=(1:18);               meas=uu(k);   % Sensor Measurements
    k=k(end)+(1);           time=uu(k);   % Simulation time, s

    % Extract mesurements
    k=1;
    pn_gps = meas(k); k=k+1; % GPS North Measurement, m
    pe_gps = meas(k); k=k+1; % GPS East Measurement, m
    alt_gps= meas(k); k=k+1; % GPS Altitude Measurement, m
    Vn_gps = meas(k); k=k+1; % GPS North Speed Measurement, m/s
    Ve_gps = meas(k); k=k+1; % GPS East Speed Measurement, m/s
    Vd_gps = meas(k); k=k+1; % GPS Downward Speed Measurement, m/s
    p_gyro = meas(k); k=k+1; % Gyro Body Rate Meas. about x, rad/s
    q_gyro = meas(k); k=k+1; % Gyro Body Rate Meas. about y, rad/s
    r_gyro = meas(k); k=k+1; % Gyro Body Rate Meas. about z, rad/s
    ax_accel = meas(k); k=k+1; % Accelerometer Meas along x, m/s/s
    ay_accel = meas(k); k=k+1; % Accelerometer Meas along y, m/s/s
    az_accel = meas(k); k=k+1; % Accelerometer Meas along z, m/s/s
    static_press = meas(k); k=k+1; % Static Pressure Meas., N/m^2
    diff_press = meas(k); k=k+1; % Differential Pressure Meas., N/m^2
    psi_mag = meas(k); k=k+1; % Yaw Meas. from Magnetometer, rad
    future_use = meas(k); k=k+1;
    future_use = meas(k); k=k+1;
    future_use = meas(k); k=k+1;

    % Filter raw measurements
    persistent lpf_static_press ...
               lpf_diff_press ...
               lpf_p_gyro ...
               lpf_q_gyro ...
               lpf_r_gyro ...
               lpf_psi_mag
    if(time==0)
        % Filter initializations
        lpf_static_press = static_press;
        lpf_diff_press = diff_press;
        lpf_p_gyro = p_gyro;
        lpf_q_gyro = q_gyro;
        lpf_r_gyro = r_gyro;
        lpf_psi_mag = psi_mag;
    end
    % NOTE: You need to modify LPF(), see below end of this file.
    lpf_static_press = LPF(static_press,lpf_static_press,P.tau_static_press,P.Ts);
    lpf_diff_press   = LPF(diff_press,lpf_diff_press,P.tau_diff_press,P.Ts);
    lpf_p_gyro = LPF(p_gyro,lpf_p_gyro,P.tau_gyro,P.Ts);
    lpf_q_gyro = LPF(q_gyro,lpf_q_gyro,P.tau_gyro,P.Ts);
    lpf_r_gyro = LPF(r_gyro,lpf_r_gyro,P.tau_gyro,P.Ts);
    lpf_psi_mag = LPF(psi_mag,lpf_psi_mag,P.tau_mag,P.Ts);
    
    % Estimate barometric altitude from static pressure
    P0 = 101325;  % Standard pressure at sea level, N/m^2
    R = 8.31432;  % Universal gas constant for air, N-m/(mol-K)
    M = 0.0289644;% Standard molar mass of atmospheric air, kg/mol
    T = 5/9*(P.air_temp_F-32)+273.15; % Air temperature in Kelvin
    Plaunch = P0*exp((-M*P.gravity)/(R*T)*(P.h0_ASL));
    h_baro = ((-R*T)/(M*P.gravity))*log(lpf_static_press/(Plaunch)); % Altitude estimate using Baro altimeter, meters above h0_ASL

    % Estimate airspeed from pitot tube differential pressure measurement
    Va_pitot = sqrt((2*lpf_diff_press)/P.rho); % Airspeed estimate using pitot tube, m/s

    % EKF to estimate roll and pitch attitude
    sigma_ekfInitUncertainty_att = [5; 5]; % nx1 (units of states)
    sigma_ekfProcessNoise_att = [P.sigma_noise_gyro; P.sigma_noise_gyro]; % nx1 (units of states)
    sigma_ekfMeasNoise_att = [P.sigma_noise_accel; P.sigma_noise_accel; P.sigma_noise_accel];
    Q_att = diag(sigma_ekfProcessNoise_att.^2); % Continuous-time process noise matrix
    m_R_att = 15;
    R_att = 10^m_R_att * diag(sigma_ekfMeasNoise_att.^2); % Measurement noise covariance matrix
    persistent xhat_att P_att
    if(time==0)
        xhat_att=[0;0]; % States: [phi; theta]
        P_att=diag(sigma_ekfInitUncertainty_att.^2);
    end
    N=10; % Number of sub-steps for propagation each sample period
    for i=1:N % Prediction step (N sub-steps)
        phi_hat=xhat_att(1);
        theta_hat=xhat_att(2);
        f_att = [1 sin(phi_hat)*tan(theta_hat) cos(phi_hat)*tan(theta_hat);...
                 0 cos(phi_hat), -sin(phi_hat)]*[p_gyro;q_gyro;r_gyro] + P.sigma_noise_gyro; % State derivatives, xdot = f(x,...)
        A_att = [q_gyro*cos(phi_hat)*tan(theta_hat)-r_gyro*sin(phi_hat)*tan(theta_hat), (q_gyro*sin(phi_hat)+r_gyro*cos(phi_hat))*(1+tan(theta_hat)^2);...
                 -q_gyro*sin(phi_hat)-r_gyro*cos(phi_hat), 0]; % Linearization (Jacobian) of f(x,...) wrt x
        xhat_att = xhat_att + (P.Ts/N)*f_att; % States propagated to sub-step N
        P_att = P_att + (P.Ts/N)*(A_att*P_att + P_att*A_att' + Q_att); % Covariance matrix propagated to sub-step N
        P_att = real(.5*P_att + .5*P_att'); % Make sure P stays real and symmetric
    end
    y_att = [p_gyro; q_gyro; r_gyro]; % Vector of actual measurements
    h_att = [q_gyro*Va_pitot*sin(theta_hat)+ P.gravity*sin(theta_hat);
             r_gyro*Va_pitot*cos(theta_hat)-p_gyro*Va_pitot*sin(theta_hat)-P.gravity*cos(theta_hat)*sin(phi_hat);...
             -q_gyro*Va_pitot*cos(theta_hat)-P.gravity*cos(theta_hat)*cos(phi_hat)]; % Mathematical model of measurements based on xhat
    C_att = [0, q_gyro*Va_pitot*cos(theta_hat)+P.gravity*cos(theta_hat);...
             -P.gravity*cos(phi_hat)*cos(theta_hat), -r_gyro*Va_pitot*sin(theta_hat)-p_gyro*Va_pitot*cos(theta_hat)+P.gravity*sin(phi_hat)*sin(theta_hat);...
             P.gravity*sin(phi_hat)*cos(theta_hat), q_gyro*Va_pitot*sin(theta_hat)+P.gravity*cos(phi_hat)*sin(theta_hat)]; % Linearization (Jacobian) of h(x,...) wrt x
    L_att = (P_att*C_att')/(C_att*P_att*C_att' + R_att); % Kalman Gain matrix
    P_att = (eye(length(xhat_att)) - L_att*C_att)*P_att; % Covariance matrix updated with measurement information
    xhat_att = xhat_att + L_att*(y_att-h_att); % States updated with measurement information
    xhat_att = mod(xhat_att+pi,2*pi)-pi; % xhat_att are attitudes, make sure they stay within +/-180degrees 
    phi_hat_unc   = sqrt(P_att(1,1)); % EKF-predicted uncertainty in phi estimate, rad 
    theta_hat_unc = sqrt(P_att(2,2)); % EKF-predicted uncertainty in theta estimate, rad 

    % Use GPS for NE position, and ground velocity vector
    
    % estimate states
    pn_hat    = 0;
    pe_hat    = 0;
    h_hat     = h_baro;
    Va_hat    = Va_pitot;
    phi_hat   = xhat_att(1);
    theta_hat = xhat_att(2);
    psi_hat   = lpf_psi_mag;
    p_hat     = lpf_p_gyro;
    q_hat     = lpf_q_gyro;
    r_hat     = lpf_r_gyro;
    Vn_hat    = 0;
    Ve_hat    = 0;
    Vd_hat    = 0;
    wn_hat    = 0;
    we_hat    = 0;
    
    % Compile output vector
    out = [...
            pn_hat;...    % 1
            pe_hat;...    % 2
            h_hat;...     % 3
            Va_hat;...    % 4
            phi_hat;...   % 5
            theta_hat;... % 6
            psi_hat;...   % 7
            p_hat;...     % 8
            q_hat;...     % 9 
            r_hat;...     % 10
            Vn_hat;...    % 11
            Ve_hat;...    % 12
            Vd_hat;...    % 13
            wn_hat;...    % 14
            we_hat;...    % 15
            phi_hat_unc;...   % 16
            theta_hat_unc;... % 17
            0; % future use
            0; % future use
            0; % future use
            0; % future use
            0; % future use
            0; % future use
        ]; % Length: 23
    
end 

function y = LPF(u,yPrev,tau,Ts)
%
%  Y(s)       a           1
% ------ = ------- = -----------,  tau: Filter time contsant, s
%  U(s)     s + a     tau*s + 1         ( tau = 1/a )
%

alpha_LPF = exp(-Ts/tau);
y = alpha_LPF*yPrev + (1-alpha_LPF)*u;

end
