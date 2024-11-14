function P = compute_autopilot_gains(models,P)
% Compute the autopilot gains that will be used in uavsim.
%
%   P = compute_autopilot_gains(models,P)
%
%   Inputs:
%      models:  Structure containing resulting simplified tranfer function
%               models, as well as coefficients used to create the TF
%               models.
%      P:       uavsim parameter structure
%
%   Outputs:
%      P:       uavsim parameter structure containing autopilot gains
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   

    %% select gains for roll loop

        % Roll Loop Design Parameters
        e_phi_max = deg2rad(45); % rad, Amount of roll error which causes saturation
        zeta_roll = 0.9; % Roll loop damping coefficient
    
        % Use described method to develop roll gains
        % Note: 
        %       P.delta_a_max is the max aileron deflection.
        %       models.a_phi1 and models.a_phi2 are the linear design
        %       model coefficients.
        P.roll_kp = (P.delta_a_max/e_phi_max) * sign(models.a_phi2);
        w_n_phi = sqrt(P.roll_kp*models.a_phi2);
        P.roll_kd = (2*zeta_roll*w_n_phi - models.a_phi1)/models.a_phi2;
        P.roll_ki = 0;

    %% select gains for course loop
        % Roll Loop Design Parameters
        zeta_course = 0.7; % Roll loop damping coefficient
        W_x = 4;
        W_n_course = w_n_phi/W_x;
        P.course_kp = (2*zeta_course*W_n_course*P.Va0)/P.gravity;
        P.course_ki = (W_n_course^2*P.Va0)/P.gravity;
        P.course_kd = 0;

    %% select gains for sideslip hold
    
        % Simulated UAV doesn't have a rudder to control
        
    %% select gains for the pitch loop, including DC gain
    % Pitch Loop Design Parameters
       e_theta_max = deg2rad(30); % rad, Amount of roll error which causes saturation
       zeta_theta = 0.9; % Roll loop damping coefficient
       
       P.pitch_kp = (P.delta_e_max/e_theta_max) * sign(models.a_theta3);
       w_n_theta = sqrt(models.a_theta2+P.pitch_kp*models.a_theta3);
       P.pitch_ki = 0;
       P.pitch_kd = (2*zeta_theta*w_n_theta - models.a_theta1)/models.a_theta3;
       P.K_theta_DC = (P.pitch_kp*models.a_theta3)/(models.a_theta2 + P.pitch_kp*models.a_theta3);

    %% select gains for altitude loop
    % Altitude Loop Design Parameters
       zeta_altitude = 0.8; % Roll loop damping coefficient
       W_h = 28;
       w_n_altitude = w_n_theta/W_h;
       P.altitude_kp = (2*zeta_altitude*w_n_altitude)/(P.K_theta_DC*P.Va0);
       P.altitude_ki = w_n_altitude^2/(P.K_theta_DC*P.Va0);
       P.altitude_kd = 0;

    %% airspeed hold using throttle
       zeta_airspeed = 1;
       W_v = 40;
       w_n_airspeed = w_n_theta/W_v;

       P.airspeed_throttle_kp = (2*zeta_airspeed*w_n_airspeed - models.a_V1)/models.a_V2;
       P.airspeed_throttle_ki = w_n_airspeed^2/models.a_V2;
       P.airspeed_throttle_kd = 0;

    %% airspeed hold using pitch
       zeta_airspeed2 = 1;
       W_v2 = 40;
       w_n_airspeed2 = w_n_theta/W_v2;
       
       P.airspeed_pitch_kp = (models.a_V1 - 2*zeta_airspeed2*w_n_airspeed2)/(P.K_theta_DC*P.gravity);
       P.airspeed_pitch_ki = -w_n_airspeed2^2/(P.K_theta_DC*P.gravity);
       P.airspeed_pitch_kd = 0;
end