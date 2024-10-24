% phi_rad = 14*pi/180;
% theta_rad = -30*pi/180;
% psi_rad = 160*pi/180; 
% 
% R_ned2b = eulerToRotationMatrix(phi_rad,theta_rad,psi_rad);
% [phi theta psi] = rotationMatrixToEuler(R_ned2b);
% phi = phi*(180/pi);
% theta = theta*(180/pi);
% psi = psi*(180/pi);
% 
% 
% va =  [20 -2 3]';
% [v alph bet] = makeVaAlphaBeta(va);
% alph = alph*(180/pi);
%bet = bet*(180/pi);

vg = [ -15 -12 2]';
[v2 gamma course] = makeVgGammaCourse(vg);
gamma = gamma*(180/pi);
course = course*(180/pi);
% 
% vg = [-15 -12 2]';
% vw = [3 4 0]';
% va = vg - vw;
% 
% [v alph bet] = makeVaAlphaBeta(va);