function [xout, yout] = joystick(ax)
% Create a virtual joystick.
% [xout yout] = joystick(ax)
% Example:
%   figure; 
%   subplot(121); 
%   xVals=nan; yVals=nan; 
%   h=plot(xVals,yVals); 
%   axis([-1 1 -1 1]); 
%   tic; 
%   while(toc<20); 
%       [x y]=joystick(subplot(224)); 
%       xVals(end+1)=x; yVals(end+1)=y; 
%       set(h,'xdata',xVals,'ydata',yVals); 
%       drawnow; 
%   end
%
% Developed for JHU EP 525.461, UAV Systems & Control
% - Barton

if strcmp(get(ax,'tag'),'joystick')

    udata=get(ax,'userdata');
    xoutOld=get(udata.stick_point,'xdata');
    youtOld=get(udata.stick_point,'ydata');
    currPt = get(ax,'currentpoint');
    
    if strcmp(get(get(ax,'parent'),'windowbuttonmotionfcn'),'allow_figure_motion(''MOTION'')') ...
        && (xoutOld~=0 || youtOld~=0 || (abs(currPt(1,1))<=1 && abs(currPt(1,2))<=1))
        
        % Here is where we limit the joystick to the square
        if abs(currPt(1,1))<=2e10 && abs(currPt(1,2))<=2e10 
            xout = min(1,max(-1,currPt(1,1)));
            yout = min(1,max(-1,currPt(1,2)));
        else
            xout=0;
            yout=0;
        end
        
        udata=get(ax,'userdata');
        set(udata.stick_line,'xdata',[0 xout],'ydata',[0 yout]);
        set(udata.stick_point,'xdata',[xout],'ydata',[yout]);
        
    else
        xout = 0;
        yout = 0;
        
        udata=get(ax,'userdata');
        set(udata.stick_line,'xdata',[0 xout],'ydata',[0 yout]);
        set(udata.stick_point,'xdata',[xout],'ydata',[yout]);        
    end
    
else
    
    cla; 
    th_circ=linspace(-pi,pi,100);
    fill(sqrt(2)*cos(th_circ),sqrt(2)*sin(th_circ),'k'); 
    axis equal; 
    axis off; 
    axis(sqrt(2)*[-1 1 -1 1]); 
    hold on; 
    fill([-1 1 1 -1],[-1 -1 1 1],'w'); 
    
    stick_horz_value = 0;
    stick_vert_value = 0;
    plot(0,0,'ko','linewidth',2,'markersize',10,'markerfacecolor',[0 0 0]);
    udata.stick_line=plot([0 stick_horz_value],[0 stick_vert_value],'k','linewidth',10);
    udata.stick_point=plot(stick_horz_value,stick_vert_value,'ko-','linewidth',2,'markersize',20,'markerfacecolor',[.5 .5 .5]);
    udata.mode='up';
    
    set(ax,'userdata',udata);
    allow_figure_motion(get(ax,'parent'));
    
    set(ax,'tag','joystick');
    
    if nargout>0
        xout=0;
        yout=0;
    end

end


