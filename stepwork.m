% s = tf('s')
% Gplant = (-5*(s+3))/((s+1.373)*(s^2+1.627*s+32.77));
% kp = -12;
% ki = -15;
% kd = -3;
% tf = PI_rateFeedback_TF(Gplant,kp,ki,kd);
% figure();
% step(6*tf)
% grid on;


A = [-3 2 0; -10 0 -15; 0 1 0];
B = [0;-5;0];
C = [0 0 1];
D = 0;

% Initial state value
x = [0; 0; 0]; % x is 2x1
% Time starts at zero,
% increments by dt sec
t = 0;
dt = 0.001;
% For retaining history
tHistory = [];
xHistory = [];
yHistory = [];
uHistory = [];

% Loop through time, stopping after 5 seconds
while t < 5
theta_c = 6; % Pitch command (degrees)
theta = x(3); % Current pitch response, deg (feedback)
theta_dot = x(2); % Pitch rate, deg/s (rate feedback)
u = PI_rateFeedback_controller(theta_c,theta,theta_dot,t==0,dt);

xdot = A*x + B*u; % Define state derivatives
x = x + xdot*dt; % Propagate state (simple Euler integration)
y = C*x + D*u; % Output is a combination of x and u
t = t + dt; % Increment time
tHistory(end+1) = t; % Retain history
xHistory(end+1,:) = x';
yHistory(end+1) = y;
uHistory(end+1) = u;
end
figure();

plot(tHistory,yHistory);
title("pitch history");
grid on;

figure();

plot(tHistory,uHistory);
title("elevator angle");
grid on;