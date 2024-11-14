% figure()
% plot(out.time_s,out.yaw_deg)
% hold on
% plot(out.time_s,out.yaw_mag_deg)
% hold on
% plot(out.time_s,out.yaw_est_deg)
% hold on
% legend('yaw_deg','yaw_mag_deg','yaw_est_deg','Interpreter', 'none')
% xlabel('Time (s)');
% ylabel('Yaws (deg)');
% title('Yaw Compare');

figure()
plot(out.time_s,out.airspeed_mps)
hold on
plot(out.time_s,out.airspeed_pitot_mps/2)
hold on
legend('airspeed_mps','airspeed_pitot_mps','Interpreter', 'none')
xlabel('Time (s)');
ylabel('Airspeed (mps)');
title('Airspeed Compare');

