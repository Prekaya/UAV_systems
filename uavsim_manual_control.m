% uavsim_manual_control.m
%
% Flight control logic for uavsim
%
% Inputs:
%   Time
%
% Outputs:
%   Control surface commands
%
% Developed for JHU EP 525.461, UAV Systems & Control
% Adapted from design project in "Small Unmanned Aircraft: Theory and
% Practice", RWBeard & TWMcClain, Princeton Univ. Press, 2012
%   
function delta = uavsim_manual_control(time,P)

    % Initialize joystick subplot
    persistent hJoystick 
    persistent hRudderThrottle delta_r delta_t
    init=0;
    if time==0 || ~ishandle(hJoystick)
        figure(461);
        hJoystick=subplot(2,6,12);
        joystick(hJoystick);
        zoom off
        pan off
        rotate3d off
        set(gcf,'windowkeypressFcn',';'); % Makes it so you can press a key and not bring up matlab terminal
        set(gcf,'currentcharacter','~'); % Using ~ as a dummy character
        delta_r=0;
        delta_t=.5;
        subplot(2,6,11);
        plot([-1 1 nan 0 0],[0 0 nan -1 1],'k--'); hold on;
        hRudderThrottle=plot(nan,nan,'ko','markersize',20,'markerfacecolor','k');
        hold off;
        title({'Elevator/Aileron: Mouse on joystick','Rudder/Throttle: use asdw keys'}) 
        axis equal; axis([-1 1 -1 1]); 
        set(gca,'xtick',[],'ytick',[]);
        xlabel('a --Rudder-- d'); ylabel('s --Throttle-- w');
    end
    
    % Acquire x & y values from mouse control on joystick subplot
    %   -1<=xval<=1, -1<=yval<=1
    [xval yval] = joystick(hJoystick);
    
    % Scale to achieve elevator and aileron
    %  (Purposefully not using full deflection range to make it easier)
    maxDeflection_deg = 15;
    delta_e = yval*maxDeflection_deg*pi/180;
    delta_a = xval*maxDeflection_deg*pi/180;
    
    % Rudder and throttle are controlled with keys
    if lower(get(461,'currentcharacter'))=='w'
        delta_t = delta_t+0.05;
        set(461,'currentcharacter','~');
    elseif lower(get(461,'currentcharacter'))=='s'
        delta_t = delta_t-0.05;
        set(461,'currentcharacter','~');
    elseif lower(get(461,'currentcharacter'))=='d'
        delta_r = delta_r+pi/180*maxDeflection_deg/20;
        set(461,'currentcharacter','~');
    elseif lower(get(461,'currentcharacter'))=='a'
        delta_r = delta_r-pi/180*maxDeflection_deg/20;
        set(461,'currentcharacter','~');
    end
    delta_t = max(0,min(1,delta_t));
    delta_r = max(-maxDeflection_deg,min(maxDeflection_deg,delta_r));
    set(hRudderThrottle,'xdata',delta_r/(maxDeflection_deg*pi/180),'ydata',2*(delta_t-.5));
    
    % Compile delta vector
    delta = [delta_e; delta_a; delta_r; delta_t];
    
    % Crude attempt to achieve real-time
    persistent tNow0 slowCount
    if time==0
        try, tNow0=toc; catch, tic; tNow0=toc; end
        slowCount=0;
    end
    while 1
        tNow=toc;
        if tNow-tNow0>time-P.Ts
            break;
        end
        if tNow-tNow0<time-1
            disp('Crude method for achieving real-time in uavsim_manual_control isn''t working');
            break;
        end
    end
    if tNow-tNow0>time+.5
        slowCount=slowCount+1;
        if mod(slowCount,100)==0
            disp('Sim is running too slow for real-time. Try adusting P.Tlog, P.Tvis, P.Ts, or config params')
        end
    end
    
end
