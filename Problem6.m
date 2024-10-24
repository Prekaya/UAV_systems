s = tf('s');
Win = 50;
Z = 0.7;
a = Win/3;

Gin = Win^2/(s^2+2*Z*Win*s+Win^2);
Gappro = a/(s+a);
Gout = a/((s/Gin) + a);
step(Gout)

subplot(2,2,1)
plot(out.time_s,out.yaw_deg)
title("Yaw vs Time")
xlabel("Time (s)")
ylabel("Yaw (deg)")

subplot(2,2,2)
plot(out.time_s,out.course_deg)
title("Course vs Time")
xlabel("Time (s)")
ylabel("Course (deg)")

subplot(2,2,3)
plot(out.time_s,out.beta_deg)
title("Sideslip vs Time")
xlabel("Time (s)")
ylabel("Sideslip (deg)")

subplot(2,2,4)
l = out.course_deg-out.yaw_deg;
plot(out.time_s,l)
title("Crab vs Time")
xlabel("Time (s)")
ylabel("Crab (deg)")