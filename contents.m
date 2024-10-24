% contents.m
%
% Listing of contents for uavsim
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

% To run simulation:
%   type 'load_uavsim'
%   Press play in Simulink
% 
% To view simulation output, use the generated "out" structure
%   e.g. plot(out.time_s,out.alt_m)
% 
% Simulink
%     uavsim.mdl - Simulink model (Architecture provided. Students will develop code.)
% 
% mfiles to setup simulation parameters
% 
%     load_uavsim.m        - Initializing routine for uavsim.  Re-running will re-initialize parameters.
%     init_uavsim_params.m - Initializes parameters (P structure)
%     uav_props_website.m  - Vehicle-specific parameters (P structure)
% 
% mfiles used as interpreted matlab functions in uavsim
% (all functions are of the format: out = uavsim_xyz(u,P))
% 
%     uavsim_kin_dyn.m         - "Kinematics & Dynamics" block
%     uavsim_forces_moments.m  - "Force & Moments" block
%     uavsim_control.m         - "Autopilot: Flight Control" block
%     uavsim_sensors.m         - "Sensors" block
%     uavsim_estimates.m       - "Autopilot: State Estimation" block
%     uavsim_truth_feedback.m  - "Truth Feedback for Autopilot" block
%     uavsim_wind.m            - "Winds" block
%     uavsim_display.m         - Simulation visualization in "Visualization & Output" block
%     uavsim_logging.m         - Simulation logging in "Visualization & Output" block
% 
% uavsim utilities
% 
%     compute_longitudinal_trim.m  - Function to determine longitudinal trim condition
%     linearize_uavsim.m           - Function to generate linear state space matrices (A & B) 
%     uavsim_manual_control.m      - Used in uavsim_control.m to control uav with a virtual joystick 
%     compute_tf_models.m          - Function to compute simplified transfer functions
%     compute_autopilot_gains.m    - Function to generate autopilot gains
% 
% general utilities
% 
%     eulerToRotationMatrix.m      - Convert Euler angles to a Rotation Matrix
%     rotationMatrixToEuler.m      - Convert Rotation Matrix to Euler angles
%     makeVaAlphaBeta.m            - Extract Va, alpha & beta from wind-relative airspeed vector in body coords
%     makeVgGammaCourse.m          - Extract Vg, gamma & course from NED velocity vector in NED coords
%     PI_rateFeedback_TF.m         - Generate a closed-loop transfer function using "PI with rateFeedback" controller
%     PI_rateFeedback_controller.m - "PI with rateFeedback" controller implementation
%
% graphical tools
%
%     allow_figure_motion.m        - Allows current figure position to be updated during mouse movement
%     joystick.m                   - Uses allow_figure_motion to draw an active vitual joystick 
%
