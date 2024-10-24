subplot(3,1,1)
plot(out.time_s,out.alt_m);
title('Altitude vs Time');
xlabel('Time (s)');
ylabel('Altitude (m)');
% 
% 
% 
% subplot(3,1,2)
% plot(out.time_s, out.airspeed_mps);
% title('Airspeed vs Time');
% xlabel('Time (s)');
% ylabel('Airspeed (mps)');
%ylim([0,15
% 
% subplot(3,1,1)]);
% plot(out.time_s, out.alpha_deg);
% title('Angle of Attack vs Time');
% xlabel('Time (s)');
% ylabel('Angle of Attack (deg)');

subplot(3,1,2)
plot(out.time_s, out.roll_deg);
title('Roll vs Time');
xlabel('Time (s)');
ylabel('Roll (deg)');
%ylim([-0.3,0]);

subplot(3,1,3)
plot(out.time_s, out.p_dps);
title('Roll Rate vs Time');
xlabel('Time (s)');
ylabel('P (deg/s)');
%ylim([-0.01,0.01]);


%ylim([0,7]);
