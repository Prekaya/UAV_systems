% uavsim_display.m
%
% Visualization of uavsim variables
%
% Inputs:
%   Various 
%
% Outputs:
%   Creates a visualization in Figure 461
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function uavsim_display(uu,P)

    % Plotting flags
    show_text = 1;
    plot_commands = 0;
    plot_estimates= 0;
    
    % Extract variables from input vector uu
    %   uu = [x(1:12); f_and_m(1:6); wind_ned(1:3); ap_cmds(1:9); estimates(1:23); meas(1:18); time(1)];
    k=1:12;          x=uu(k);         % States
    k=k(end)+(1:6);  f_and_m=uu(k);   % Forces and Moments, body
    k=k(end)+(1:3);  wind_ned=uu(k);  % Wind vector, ned, m/s
    k=k(end)+(1:4);  deltas=uu(k);    % Control commands
    k=k(end)+(1:9);  ap_cmds=uu(k);   % Autopilot commands
    k=k(end)+(1:23); estimates=uu(k); % Autopilot state estimates
    k=k(end)+(1:18); meas=uu(k);      % Measurements
    k=k(end)+(1);    time=uu(k);      % Simulation time, s

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
    future_use   = estimates(16:23);

    % Extract a/p commands
    Va_c      = ap_cmds(1);
    h_c       = ap_cmds(2);
    chi_c     = ap_cmds(3);
    phi_c     = ap_cmds(4);
    theta_c   = ap_cmds(5);
    
    % Compute Rotation Matrices
    R_ned2b = eulerToRotationMatrix(phi,theta,psi);
    
    % Reconstruct euler angles from R_ned2b.
    %  (This is completely unnecessary, but it is here
    %  purely to verify that students have coded the
    %  Euler angle conversion routines correctly.)
    [phi, theta, psi] = rotationMatrixToEuler(R_ned2b);

    % Rotate wind vector to body frame
    wind_b = R_ned2b*wind_ned;

    % compute airspeed Va
    v_rel_b = [u;v;w]-wind_b;
    [Va, alpha, beta] = makeVaAlphaBeta(v_rel_b);
      
    % define persistent variables (graphic handles)
    persistent aircraft_handle trajectory_handle ground_handle trace_handles text_handle1 text_handle2
    persistent V F patchcolors
    
    % first time function is called, initialize plot and persistent vars
    if time==0,

        % Figure 461 in honor of class number
        figure(461)
        clf
        
        % Init aircraft drawing
        
        % Generate aircraft patch parameters
        [V, F, patchcolors] = aircraft_patch_params;
    
        subplot(1,3,2); hold on
        aircraft_handle = drawAircraftBody(V,F,patchcolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               []);
                                           
        trajectory_handle = plot3(pe,pn,-pd,'b-');
        ground_handle = plot3(pe,pn,0,'k');
        
        title('UAV not drawn to scale')
        xlabel('East')
        ylabel('North')
        zlabel('Up')
        view(35,21)  % set the view angle for figure
        hold on
        grid on
        axis equal
        set(gca,'userdata',get(gca,'position'));
        
        % Init other plots (red: command, green: estimate, blue: truth)
        
        subplot(4,3,1);
        hold on
        trace_handles.h_alt_cmd = plot(nan,nan,'r--'); 
        trace_handles.h_alt_est = plot(nan,nan,'color',[0 .5 0]);
        trace_handles.h_alt     = plot(nan,nan,'b');
        hold off
        grid on;
        xlabel('Time, s');
        ylabel('Alt, m');
        
        subplot(4,3,4);
        hold on
        trace_handles.h_speed_cmd = plot(nan,nan,'r--'); 
        trace_handles.h_speed_est = plot(nan,nan,'color',[0 .5 0]);
        trace_handles.h_speed     = plot(nan,nan,'b');
        grid on;
        xlabel('Time, s');
        ylabel('Airspeed, m/s');
        
        subplot(4,3,7);
        hold on
        trace_handles.h_pitch_cmd = plot(nan,nan,'r--'); 
        trace_handles.h_pitch_est = plot(nan,nan,'color',[0 .5 0]);
        trace_handles.h_pitch     = plot(nan,nan,'b');
        grid on;
        grid on;
        xlabel('Time, s');
        ylabel('Pitch, deg');
        
        subplot(4,3,10);
        hold on
        trace_handles.h_roll_cmd = plot(nan,nan,'r--'); 
        trace_handles.h_roll_est = plot(nan,nan,'color',[0 .5 0]);
        trace_handles.h_roll     = plot(nan,nan,'b');
        grid on;
        grid on;
        xlabel('Time, s');
        ylabel('Roll, deg');        

        text_handle1=uicontrol('parent',461,'style','text','string','', ...
            'units','normalized','position',[0.6682+0.00    0.5838    0.16    0.3412], ...
            'horizontalalignment','left');
        text_handle2=uicontrol('parent',461,'style','text','string','', ...
            'units','normalized','position',[0.6682+0.16    0.5838    0.16    0.3412], ...
            'horizontalalignment','left');
    end
    
    % Update graphics data

    % Update aircraft on 3D plot
    h3d=get(aircraft_handle,'parent');
    drawAircraftBody(V,F,patchcolors,...
                       pn,pe,pd,phi,theta,psi,...
                       aircraft_handle);
    zoom_width_m=60;
    zlim = [0 -pd+zoom_width_m/2]; 
    zlim(1) = max(0,zlim(2)-3*zoom_width_m);
    axis(h3d,[pe-zoom_width_m/2,pe+zoom_width_m/2,pn-zoom_width_m/2,pn+zoom_width_m/2,zlim]);
    set(h3d,'position',get(h3d,'userdata'));

    % Update positions on 3D plot
    xdata=[get(trajectory_handle,'xdata') pe];
    ydata=[get(trajectory_handle,'ydata') pn];
    zdata=[get(trajectory_handle,'zdata') -pd];
    set(trajectory_handle,'xdata',xdata,'ydata',ydata,'zdata',zdata);        
    xdata=[get(ground_handle,'xdata') pe];
    ydata=[get(ground_handle,'ydata') pn];
    zdata=[get(ground_handle,'zdata') 0];
    set(ground_handle,'xdata',xdata,'ydata',ydata,'zdata',zdata);

    % Concatenate data to update truth traces
    concatDataToPlotHandle(trace_handles.h_alt,time,-pd);
    concatDataToPlotHandle(trace_handles.h_speed,time,Va);
    concatDataToPlotHandle(trace_handles.h_pitch,time,theta*180/pi);
    concatDataToPlotHandle(trace_handles.h_roll,time,mod(phi*180/pi+180,360)-180);

    % Concatenate data to update estimate traces
    if(plot_estimates)
        concatDataToPlotHandle(trace_handles.h_alt_est,time,h_hat);
        concatDataToPlotHandle(trace_handles.h_speed_est,time,Va_hat);
        concatDataToPlotHandle(trace_handles.h_pitch_est,time,theta_hat*180/pi);
        concatDataToPlotHandle(trace_handles.h_roll_est,time,mod(phi_hat*180/pi+180,360)-180);
    end
    
    % Concatenate data to update estimate traces
    if(plot_commands)
        concatDataToPlotHandle(trace_handles.h_alt_cmd,time,h_c);
        concatDataToPlotHandle(trace_handles.h_speed_cmd,time,Va_c);
        concatDataToPlotHandle(trace_handles.h_pitch_cmd,time,theta_c*180/pi);
        concatDataToPlotHandle(trace_handles.h_roll_cmd,time,mod(phi_c*180/pi+180,360)-180);
    end
    
    % Display textual information
    if show_text
        set(text_handle1,'string', { ...
            sprintf('time: %5.1f s',time), ...
            sprintf(''), ...
            sprintf('Alt: %5.1f m',-pd), ...
            sprintf('Vg: %5.1f m/s',sqrt(u^2+v^2+w^2)), ...
            sprintf(''), ...
            sprintf('Wind'), ...
            sprintf(' N: %4.1f m/s',wind_ned(1)), ...
            sprintf(' E: %4.1f m/s',wind_ned(2)), ...
            sprintf(' D: %4.1f m/s',wind_ned(3)), ...
            });
        set(text_handle2,'string', { ...
            sprintf('alpha: %6.2f deg',alpha*180/pi), ...
            sprintf('beta: %6.2f deg',beta*180/pi), ...
            sprintf(''), ...
            sprintf('de: %6.2f deg',deltas(1)*180/pi), ...
            sprintf('da: %6.2f deg',deltas(2)*180/pi), ...
            sprintf('dr: %6.2f deg',deltas(3)*180/pi), ...
            sprintf('dt: %6.2f (0-1)',deltas(4)), ...
            });
    end

    % Add time to title
    persistent tNow0
    if time==0
        try, tNow0=toc; catch, tic; tNow0=toc; end
    end
    set(461,'name',sprintf('UAVSIM --  Sim Time: %.1f, Real Time: %.1f',time,toc-tNow0));
    
    % Force a drawnow each time
    drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Concatenate (X,Y) or (X,Y,Z) data onto existing figure handle
function handle = concatDataToPlotHandle(handle,newX,newY,newZ)

    if ~exist('newZ','var')
        xdata=[get(handle,'xdata') newX];
        ydata=[get(handle,'ydata') newY];
        set(handle,'xdata',xdata,'ydata',ydata);
    else
        xdata=[get(handle,'xdata') newX];
        ydata=[get(handle,'ydata') newY];
        zdata=[get(handle,'zdata') newZ];
        set(handle,'xdata',xdata,'ydata',ydata,'zdata',zdata);
    end    

end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% drawAircraft
% return new handle if handle argument is empty
function handle = drawAircraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle)
    V = rotate(V', phi, theta, psi)';  % rotate aircraft
    V = translate(V', pn, pe, pd)';  % translate aircraft
    % transform vertices from NED to XYZ (for matlab rendering)
    R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
    V = V*R;

    if isempty(handle),
        handle = patch('Vertices', V, 'Faces', F,...
                     'FaceVertexCData',patchcolors,...
                     'FaceColor','flat');
    else
        set(handle,'Vertices',V,'Faces',F);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
    % define rotation matrix
    R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
    R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
    R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
    R = R_yaw*R_pitch*R_roll;
    % rotate vertices
    XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
	XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make parameters for drawing aircraft
function [V, F, colors] = aircraft_patch_params
    % V: Vertices
    % F: Faces
    % colors: face colors

    % parameters for drawing aircrafts  
    size = 1; % scale size
    fuse_l1    = 7;
    fuse_l2    = 4;
    fuse_l3    = 15;
    fuse_w     = 2;
    wing_l     = 6;
    wing_w     = 20;
    tail_l     = 3;
    tail_h     = 3;
    tailwing_w = 10;
    tailwing_l = 3;
    % colors
    red     = [1, 0, 0];
    green   = [0, 1, 0];
    blue    = [0, 0, 1];
    yellow  = [1,1,0];
    magenta = [0, 1, 1];


    % define vertices and faces for aircraft
    V = [...
    fuse_l1,             0,             0;...        % point 1
    fuse_l2,            -fuse_w/2,     -fuse_w/2;... % point 2     
    fuse_l2,             fuse_w/2,     -fuse_w/2;... % point 3     
    fuse_l2,             fuse_w/2,      fuse_w/2;... % point 4
    fuse_l2,            -fuse_w/2,      fuse_w/2;... % point 5
    -fuse_l3,             0,             0;...        % point 6
    0,                   wing_w/2,      0;...        % point 7
    -wing_l,              wing_w/2,      0;...        % point 8
    -wing_l,             -wing_w/2,      0;...        % point 9
    0,                  -wing_w/2,      0;...        % point 10
    -fuse_l3+tailwing_l,  tailwing_w/2,  0;...        % point 11
    -fuse_l3,             tailwing_w/2,  0;...        % point 12
    -fuse_l3,            -tailwing_w/2,  0;...        % point 13
    -fuse_l3+tailwing_l, -tailwing_w/2,  0;...        % point 14
    -fuse_l3+tailwing_l,  0,             0;...        % point 15
    -fuse_l3+tailwing_l,  0,             -tail_h;...  % point 16
    -fuse_l3,             0,             -tail_h;...  % point 17
    ];

    F = [...
        1,  2,  3,  1;... % nose-top
        1,  3,  4,  1;... % nose-left
        1,  4,  5,  1;... % nose-bottom
        1,  5,  2,  1;... % nose-right
        2,  3,  6,  2;... % fuselage-top
        3,  6,  4,  3;... % fuselage-left
        4,  6,  5,  4;... % fuselage-bottom
        2,  5,  6,  2;... % fuselage-right
        7,  8,  9, 10;... % wing
       11, 12, 13, 14;... % tailwing
        6, 15, 17, 17;... % tail

    ];  

    colors = [...
        yellow;... % nose-top
        yellow;... % nose-left
        yellow;... % nose-bottom
        yellow;... % nose-right
        blue;... % fuselage-top
        blue;... % fuselage-left
        red;... % fuselage-bottom
        blue;... % fuselage-right
        green;... % wing
        green;... % tailwing
        blue;... % tail
    ];

    V = size*V;   % rescale vertices
end
