syms pn_hat pe_hat pd_hat p_gyro q_gyro r_gyro vn_hat ve_hat vd_hat phi_hat theta_hat psi_hat Va g ax_accel ay_accel az_accel g
%     f_att=[p_gyro+q_gyro*sin(phi_hat)*tan(theta_hat)+r_gyro*cos(phi_hat)*tan(theta_hat); ... % Define f
%            q_gyro*cos(phi_hat)-r_gyro*sin(phi_hat)];
% 
%     A_att = [diff(f_att,phi_hat), diff(f_att,theta_hat)];% Linearization (Jacobian) of f(x,...) wrt x
% 
% 
%     h_att = [q_gyro*Va*sin(theta_hat)+g*sin(theta_hat);
%          r_gyro*Va*cos(theta_hat)-p_gyro*Va*sin(theta_hat)-g*cos(theta_hat)*sin(phi_hat);...
%          -q_gyro*Va*cos(theta_hat)-g*cos(theta_hat)*cos(phi_hat)]; % Mathematical model of measurements based on xhat
% 
% % Define C: Linearization (Jacobian) of h(xhat,...) wrt xhat (mxn matrix)
% C_att = [diff(h_att,phi_hat), diff(h_att,theta_hat)]
% 
R_ned2b = eulerToRotationMatrix(phi_hat,theta_hat,psi_hat);
        % NED Position EoMs
 bottom = R_ned2b*[ax_accel; ay_accel; az_accel] + [0; 0; g];
 f_gps = [vn_hat; ve_hat; vd_hat; bottom];

[diff(f_gps, pn_hat) diff(f_gps, pe_hat) diff(f_gps, pd_hat) diff(f_gps, vn_hat) diff(f_gps, ve_hat) diff(f_gps, vd_hat)]


