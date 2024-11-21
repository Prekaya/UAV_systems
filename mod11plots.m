% 
% figure()
% plot(out.time_s,out.pitch_deg)
% hold on
% plot(out.time_s,out.pitch_est_deg)
% legend('Pitch\_deg','Pitch\_est\_deg')
% xlabel('Time (s)');
% ylabel('Pitch (deg)');
% title('Pitch Compare');
% 
figure()
plot(out.time_s,out.roll_deg)
hold on
plot(out.time_s,out.roll_est_deg)
legend('Roll\_deg','Roll\_est\_deg')
xlabel('Time (s)');
ylabel('Roll (deg)');
title('Roll Compare');

figure()
plot(out.time_s, deg2rad(out.pitch_est_deg-out.pitch_deg), ...
out.time_s, +out.theta_hat_unc, out.time_s, -out.theta_hat_unc)

figure()
plot(out.time_s, deg2rad(out.roll_est_deg-out.roll_deg), ...
out.time_s, +out.phi_hat_unc, out.time_s, -out.phi_hat_unc)

% figure()
% plot(out.time_s,out.alt_m)
% hold on
% plot(out.time_s,out.alt_gps_m)
% hold on
% plot(out.time_s,out.alt_cmd_m, 'Color','r','LineStyle','--')
% legend('Alt\_m','Alt\_gps\_m','Alt\_cmd\_m')
% xlabel('Time (s)');
% ylabel('Alt (m)');
% title('Alt Compare');
% 
% figure()
% plot(out.time_s,out.airspeed_mps)
% hold on
% plot(out.time_s,out.airspeed_pitot_mps)
% hold on
% plot(out.time_s,out.airspeed_cmd_mps, 'Color','r','LineStyle','--')
% legend('Airspeed\_mps','Airspeed\_gps\_mps','Airspeed\_cmd\_mps')
% xlabel('Time (s)');
% ylabel('Airspeed (mps)');
% title('Airspeed Compare');
% 
% figure()
% plot(out.time_s,out.course_deg)
% hold on
% plot(out.time_s,out.course_gps_deg)
% hold on
% plot(out.time_s,out.course_cmd_deg, 'Color','r','LineStyle','--')
% legend('Course\_deg','Course\_est\_deg','Course\_cmd\_deg')
% xlabel('Time (s)');
% ylabel('Course (deg)');
% title('Course Compare');
% 
% % figure()
% % plot(out.time_s, out.roll_est_deg-out.roll_deg, ...
% % out.time_s, +out.phi_hat_unc, out.time_s, -out.phi_hat_unc)
% % title('Roll Error');
% 
% % figure()
% % plot(out.time_s, out.pitch_est_deg-out.pitch_deg, ...
% % out.time_s, +out.theta_hat_unc, out.time_s, -out.theta_hat_unc)
% % title('Pitch Error');
