s = tf('s');
kp = 1;
ki = 2;
kd = 3;
Gpid = (kd*s^2+kp*s+ki)/(s+kd*s^2+kp*s+ki);
Gpir = (kp*s+ki)/(s+kd*s^2+kp*s+ki);

figure()
step(Gpid);
title("PID step response");

figure()
step(Gpir);
title("PIR step response");
