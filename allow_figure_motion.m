function allow_figure_motion(in)
% allow_figure_motion - allows 'currentpoint' to be updated under mouse
% movement.  (Nominally, MATLAB only updates 'currentpoint' on button
% downs.)
% Usage:  allow_figure_motion(fig), Default: fig=gcf
%
% Example: Draw curves with mouse
%
%         figure;
%         allow_figure_motion(gcf);
%         h=plot(nan,nan,'b.-');
%         axis equal;
%         axis([-1 1 -1 1]);
%         title('"Draw" with mouse, Ctrl-C to end');
%         while 1
%             xdata=get(h,'xdata');
%             ydata=get(h,'ydata');
%             v=get(gca,'currentpoint');
%             xMouse=v(1,1);
%             yMouse=v(1,2);
%             if xdata(end)~=xMouse || ydata(end)~=yMouse
%                 set(h,'xdata',[xdata xMouse],'ydata',[ydata yMouse]);
%             end
%             drawnow
%         end
%
% Developed for JHU EP 525.461, UAV Systems & Control
% - Barton

if nargin<1
    in=gcf;
end

% Prior to 2014a, gcf returned a numeric value.
% As of 2014a, gcf returns a handle structure.
% If "in" is a handle structure, reset it to the numeric handle.
if length(in)==1 && ~isnumeric(in) && ishandle(in)
    in = get(in,'Number'); 
end

if isnumeric(in)
    set(in,'windowButtonDownFcn','allow_figure_motion(''DOWN'')');
    set(in,'windowButtonMotionFcn','');    
    set(in,'windowButtonUpFcn','');    
else
    if strcmp(in,'DOWN')    
        set(gcf,'windowButtonDownFcn','');    
        set(gcf,'windowButtonMotionFcn','allow_figure_motion(''MOTION'')');
        set(gcf,'windowButtonUpFcn','allow_figure_motion(''UP'')');
    elseif strcmp(in,'MOTION')    
        % axis must be updated somehow
        set(gca,'userdata',get(gca,'userdata'));
        pause(.01);
    elseif strcmp(in,'UP')
        set(gcf,'windowButtonDownFcn','allow_figure_motion(''DOWN'')');
        set(gcf,'windowButtonMotionFcn','');    
        set(gcf,'windowButtonUpFcn','');    
    end
end


