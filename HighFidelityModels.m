%% Lateral Channel
[A B]=linearize_uavsim(P);
s=tf('s'); % Create a Laplace s variable
H = ss(A, B, eye(12), zeros(12,4) ); % Convert to Laplace transfer functions
H = minreal(H); % Use min. realization (i.e. cancel identical poles and zeros)
H = zpk(H); % Convert from polynomial num and den to zero/pole form

kpd=3; ku=4; kv=5; kw=6; kphi=7; ktheta=8; kpsi=9; kp=10; kq=11; kr=12;
kde=1; kda=2; kdr=3; kdt=4;

% Open Loop response from aileron to roll
H(kphi,kda) % Higher fidelity than models.G_da2phi
% Closed Loop response from roll command to roll
G_phic2phi = PI_rateFeedback_TF(H(kphi,kda),P.roll_kp,P.roll_ki,P.roll_kd)
% Closed Loop response from course command to course
G_chic2chi = PI_rateFeedback_TF(G_phic2phi*models.G_phi2chi, ...
P.course_kp,P.course_ki,P.course_kd);
%% Longitudinal Channel
% Open Loop response from elevator to pitch
H(ktheta,kde) % Higher fidelity than models.G_de2theta
% Closed Loop response from pitch command to pitch
G_thetac2theta = PI_rateFeedback_TF(H(ktheta,kde),P.pitch_kp,P.pitch_ki,P.pitch_kd)
% Closed Loop response from alt command to alt
G_altc2alt = PI_rateFeedback_TF(G_thetac2theta*models.G_theta2h, ...
P.altitude_kp,P.altitude_ki,P.altitude_kd)
% Closed Loop response from airspeed commmand to airspeed using pitch
G_vac2va_pitch = PI_rateFeedback_TF(G_thetac2theta*models.G_theta2Va, ...
P.airspeed_pitch_kp,P.airspeed_pitch_ki,P.airspeed_pitch_kd)
% Open Loop response from throttle to airspeed
models.G_dt2Va % Note: Can't use H(ku,kdt) because it is dominated by phugoid
% Closed Loop response from airspeed commmand to airspeed using throttle
G_vac2va_throttle = PI_rateFeedback_TF(models.G_dt2Va, ...
P.airspeed_throttle_kp,P.airspeed_throttle_ki,P.airspeed_throttle_kd)