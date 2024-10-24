% uavsim_wind.m
%
% Generation of total wind vector in NED coordinates
%
% Inputs:
%   UAV States
%   Gust vector in body coordinates
%   Time
%
% Outputs:
%   Total wind+gust vector in NED coordinates
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function out = uavsim_wind(uu, P)

    % Extract variables from input vector uu
    %   uu = [x(1:12); gust_b(1:3); time(1)];
    k=(1:12);        x=uu(k);          % states
    k=k(end)+(1:3);  gust_b=uu(k);     % Gust vector, body coordinates, m/s
    k=k(end)+(1);    time=uu(k);       % Simulation time, s

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

    % Your code goes below...
    
    % Generate steady wind vector in NED coordinates
    ws_ned = [0 5 0];

    % Compute DCMs
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);

    % compute wind vector in the inertial frame
    %   ws_ned & gust_b -> wind_ned
    wind_ned = ws_ned + R_ned2b*gust_b;

    % Compile function output
    out = [wind_ned]; % Length 3

end