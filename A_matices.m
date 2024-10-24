[A B]=linearize_uavsim(P);

% A_lon = A([4 6 11 8 3],[4 6 11 8 3]);
% B_lon = B([4 6 11 8 3],[1 4]);
% 
% A_lat = A([5 10 12 7 9],[5 10 12 7 9]);
% B_lat = B([5 10 12 7 9],[2 3]);
% 
% 
% m_de = (P.rho*P.Va0^2*P.S_wing*P.c*P.C_m_delta_e)/(2*P.Jy)
% 
% m_de = (P.rho*P.Va0^2*P.S_wing*P.c*P.C_m_delta_e)/(2*P.Jy)
% disp()
s=tf('s'); % Create a Laplace s variable
H = ss(A, B, eye(12), zeros(12,4) ); % Convert to Laplace transfer functions
H = minreal(H); % Use min. realization (i.e. cancel identical poles and zeros)
H = zpk(H); % Convert from polynomial num and den to zero/pole form

% figure(1);
% step(H(10,2),[0 5]);
% title("High Fidelity");
% figure(2);
% step(models.G_da2p,[0 5]);
% title("Low Fidelity")

% Gcl_roll_low =PI_rateFeedback_TF(models.G_da2phi, P.roll_kp,P.roll_ki,P.roll_kd);
% Gcl_roll_high =PI_rateFeedback_TF( H(7,2), P.roll_kp,P.roll_ki,P.roll_kd);
% step(Gcl_roll_low, Gcl_roll_high, 2) % 2 seconds


plot(out.time_s, out.roll_deg)
hold on
plot(out.time_s, out.da_deg)
xlabel('Time (s)');
ylabel('Roll and da (deg)');
title('Roll and Aileron ephimax:90 zetaroll:1.5')
legend('Roll','Aileron Deflection')