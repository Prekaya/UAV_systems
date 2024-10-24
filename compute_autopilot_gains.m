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
        P.roll_kd = (2*zeta_roll*sqrt(P.roll_kp*models.a_phi2) - models.a_phi1)/models.a_phi2;
        P.roll_ki = 0;

    %% select gains for course loop

        P.course_kp = nan;
        P.course_ki = nan;
        P.course_kd = nan;

    %% select gains for sideslip hold
    
        % Simulated UAV doesn't have a rudder to control
        
    %% select gains for the pitch loop, including DC gain

       P.pitch_kp = nan;
       P.pitch_ki = nan;
       P.pitch_kd = nan;
       P.K_theta_DC = nan;

    %% select gains for altitude loop

       P.altitude_kp = nan;
       P.altitude_ki = nan;
       P.altitude_kd = 0;

    %% airspeed hold using pitch

       P.airspeed_pitch_kp = nan;
       P.airspeed_pitch_ki = nan;
       P.airspeed_pitch_kd = 0;

    %% airspeed hold using throttle

       P.airspeed_throttle_kp = nan;
       P.airspeed_throttle_ki = nan;
       P.airspeed_throttle_kd = 0;

end