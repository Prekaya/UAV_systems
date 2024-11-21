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
    g = P.gravity;
    T = 5/9*(P.air_temp_F-32)+273.15; % Air temperature in Kelvin
    Plaunch = P0*exp((-M*g)/(R*T)*(P.h0_ASL));
    h_baro = ((-R*T)/(M*g))*log(lpf_static_press/(Plaunch)); % Altitude estimate using Baro altimeter, meters above h0_ASL
    
    % Estimate airspeed from pitot tube differential pressure measurement
    Va_pitot = sqrt((2*lpf_diff_press)/P.rho); % Airspeed estimate using pitot tube, m/s
%%
    % EKF to estimate roll and pitch attitude
    
    Va = Va_pitot;
    sigma_ekfInitUncertainty_att = deg2rad([5; 5]); % nx1 (units of states)
    sigma_ekfProcessNoise_att = [P.sigma_noise_gyro; P.sigma_noise_gyro]; % nx1 (units of states)
    sigma_ekfMeasNoise_att = [P.sigma_noise_accel; P.sigma_noise_accel; P.sigma_noise_accel];
    m_R_att = 5;
    persistent xhat_att P_att
    if(time==0)
        xhat_att=[0;0]; % States: [phi_hat; theta]
        P_att=diag(sigma_ekfInitUncertainty_att.^2);  % nxn (Initial P)
    end
    Q_att = diag(sigma_ekfProcessNoise_att.^2); % nxn
    R_att = 10^m_R_att * diag(sigma_ekfMeasNoise_att.^2); % mxm Measurement noise covariance matrix   

    N=10; % Number of sub-steps for propagation each sample period
    for i=1:N % Prediction step (N sub-steps)
        phi   = xhat_att(1);
        theta = xhat_att(2);
        f_att=[p_gyro+q_gyro*sin(phi)*tan(theta)+r_gyro*cos(phi)*tan(theta); ... % Define f
               q_gyro*cos(phi)-r_gyro*sin(phi)];

        A_att = [q_gyro*cos(phi)*tan(theta)-r_gyro*sin(phi)*tan(theta), (q_gyro*sin(phi) + r_gyro*cos(phi))*(1+ tan(theta)^2);...
                -r_gyro*cos(phi)-q_gyro*sin(phi),                              0]; % Linearization (Jacobian) of f(x,...) wrt x
        
        xhat_att = xhat_att + (P.Ts/N)*f_att; % States propagated to sub-step N
        
        % Covariance matrix propagated to sub-step N
        P_att = P_att + P.Ts/N*(A_att*P_att + P_att*A_att' + Q_att); % Covariance matrix propagated to sub-step N
        P_att = real(.5*P_att + .5*P_att'); % Make sure P stays real and symmetric
    end
    phi   = xhat_att(1);
    theta = xhat_att(2);
    ymeas_att = [ax_accel; ay_accel; az_accel]; % Vector of actual measurements
    
    % Mathematical model of measurements based on xhat: yhat=h(xhat,...)
    h_att = [q_gyro*Va*sin(theta)+g*sin(theta);
             r_gyro*Va*cos(theta)-p_gyro*Va*sin(theta)-g*cos(theta)*sin(phi);...
             -q_gyro*Va*cos(theta)-g*cos(theta)*cos(phi)]; % Mathematical model of measurements based on xhat
    
    % Define C: Linearization (Jacobian) of h(xhat,...) wrt xhat (mxn matrix)
    C_att = [                             0,       g*cos(theta)+Va*q_gyro*cos(theta);
            -g*cos(phi)*cos(theta), g*sin(phi)*sin(theta)-Va*r_gyro*sin(theta)-Va*p_gyro*cos(theta);
             g*cos(theta)*sin(phi),         Va*q_gyro*sin(theta)+g*cos(phi)*sin(theta)];% Linearization (Jacobian) of h(x,...) wrt x
    
    % Kalman Gain matrix, nxm
    % L: weightings to map measurement residuals into state estimates
    L_att = (P_att*C_att')/(C_att*P_att*C_att' + R_att);
    
    % Covariance matrix updated with measurement information
    I_att = eye(size(P_att));
    P_att = (I_att - L_att*C_att)*P_att; 
    P_att = real(.5*P_att + .5*P_att'); % Make sure P stays real and symmetric
    xhat_att = xhat_att + L_att*(ymeas_att-h_att); % States updated with measurement information
    xhat_att = mod(xhat_att+pi,2*pi)-pi; % xhat_att are attitudes, make sure they stay within +/-180degrees 
    phi_hat   = xhat_att(1);
    theta_hat = xhat_att(2);
    phi_unc   = rad2deg(sqrt(P_att(1,1))); % EKF-predicted uncertainty in phi estimate, rad 
    theta_unc = sqrt(P_att(2,2)); % EKF-predicted uncertainty in theta estimate, rad 

 %%
 % EKF to estimate gps position
    sigma_ekfInitUncertainty_gps = [5;5;5;2;2;2]; % nx1 (units of states)
    sigma_ekfProcessNoise_gps = [0.5; 0.5; 0.5; 0.3; 0.3; 0.3]; % nx1 (units of states)
    sigma_ekfMeasNoise_gps = [2; 2; 2; 0.1; 0.1; 0.1];
    persistent xhat_gps P_gps
    if(time==0)
        xhat_gps=[pn_gps; pe_gps; -alt_gps; Vn_gps; Ve_gps; Vd_gps]; % States: [pn; pe; pd; vn; ve; vd]
        P_gps=diag(sigma_ekfInitUncertainty_gps.^2);  % nxn (Initial P)
    end
    Q_gps = diag(sigma_ekfProcessNoise_gps.^2); % nxn
    R_gps = diag(sigma_ekfMeasNoise_gps.^2); % mxm Measurement noise covariance matrix   
    R_ned2b = eulerToRotationMatrix(phi_hat,theta_hat,psi_mag);
    N=10; % Number of sub-steps for propagation each sample period
    for i=1:N % Prediction step (N sub-steps)
        f_gps = [xhat_gps(4);xhat_gps(5);xhat_gps(6); ...
        R_ned2b'*[ax_accel;ay_accel;az_accel]+[0;0;P.gravity]];
        A_gps = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; zeros(3,6)];
        xhat_gps = xhat_gps + P.Ts/N*f_gps; % States propagated to sub-step N
        P_gps = P_gps + P.Ts/N*(A_gps*P_gps + P_gps*A_gps' + Q_gps);
        P_gps = real(.5*P_gps + .5*P_gps'); % Make sure P stays real and symmetric
    end
    persistent prev_pn_gps prev_pe_gps
    if time==0
        prev_pn_gps=0;
        prev_pe_gps=0;
    end
    if (pn_gps~=prev_pn_gps) || (pe_gps~=prev_pe_gps)
        prev_pn_gps=pn_gps;
        prev_pe_gps=pe_gps;
        y_gps = [pn_gps;pe_gps;-alt_gps;Vn_gps;Ve_gps;Vd_gps]; % Vector of actual measurements
        h_gps = xhat_gps; % Mathematical model of measurements based on xhat
        C_gps = eye(6); % Linearization (Jacobian) of h(x,...) wrt x
        L_gps = (P_gps*C_gps')/(C_gps*P_gps*C_gps'+R_gps); % Kalman Gain matrix
        P_gps = (eye(size(P_gps))-L_gps*C_gps)*P_gps;
        P_gps = real(.5*P_gps + .5*P_gps');
        xhat_gps = xhat_gps + L_gps*(y_gps-h_gps);
    end
    %phi_unc   = sqrt(P_gps(1,1)); % EKF-predicted uncertainty in phi estimate, rad 
    %theta_unc = sqrt(P_gps(2,2)); % EKF-predicted uncertainty in theta estimate, rad 
    %%

    % Use GPS for NE position, and ground velocity vector
    % estimate states
    pn_hat    = xhat_gps(1);
    pe_hat    = xhat_gps(2);
    h_hat     = h_baro;
    Va_hat    = sqrt(sum(xhat_gps(4:6).^2));
    phi_hat   = xhat_att(1);
    theta_hat = xhat_att(2);
    psi_hat   = lpf_psi_mag;
    p_hat     = lpf_p_gyro;
    q_hat     = lpf_q_gyro;
    r_hat     = lpf_r_gyro;
    Vn_hat    = xhat_gps(4);
    Ve_hat    = xhat_gps(5);
    Vd_hat    = xhat_gps(6);
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
            phi_unc;...   % 16
            theta_unc;... % 17
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
