% load_uavsim.m
%
% Initializer for uavsim.mdl.
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012

% Bring up simulink model
open('uavsim')

% Load UAV parameters
P = init_uavsim_params;

% Compute the trim condition and set trim parameters in P
% (Uncomment when necessary)
P = compute_longitudinal_trim(P);

% Generate linear response models to be used in autopilot development
% (Uncomment when necessary)
models = compute_tf_models(P);

% Compute autopilot gains
% (Uncomment when necessary)
P = compute_autopilot_gains(models,P);
