% subplot(3,1,1)
% plot(out.time_s,out.alt_m);
% title('Altitude vs Time');
% xlabel('Time (s)');
% ylabel('Altitude (m)');
% % 
% % 
% % 
% % subplot(3,1,2)
% % plot(out.time_s, out.airspeed_mps);
% % title('Airspeed vs Time');
% % xlabel('Time (s)');
% % ylabel('Airspeed (mps)');
% %ylim([0,15
% % 
% % subplot(3,1,1)]);
% % plot(out.time_s, out.alpha_deg);
% % title('Angle of Attack vs Time');
% % xlabel('Time (s)');
% % ylabel('Angle of Attack (deg)');
% 
% subplot(3,1,2)
% plot(out.time_s, out.roll_deg);
% title('Roll vs Time');
% xlabel('Time (s)');
% ylabel('Roll (deg)');
% %ylim([-0.3,0]);
% 
% subplot(3,1,3)
% plot(out.time_s, out.p_dps);
% title('Roll Rate vs Time');
% xlabel('Time (s)');
% ylabel('P (deg/s)');
% %ylim([-0.01,0.01]);
% 
% 
% %ylim([0,7]);

figure()
plot(out.time_s,out.alt_cmd_m)
hold on
plot(out.time_s,out.alt_m)
hold on
xlabel('Time (s)');
legend('Altitude cmd','Altitude')
title('Altitude command and response')
saveas(gcf,'Altitude command and response.png')

figure()
plot(out.time_s,out.airspeed_cmd_mps)
hold on
plot(out.time_s,out.airspeed_mps)
hold on
xlabel('Time (s)');
legend('Airspeed cmd','Airspeed')
title('Airspeed command and response')
saveas(gcf,'Airspeed command and response.png')

figure()
plot(out.time_s,out.pitch_cmd_deg)
hold on
plot(out.time_s,out.pitch_deg)
hold on
xlabel('Time (s)');
legend('Pitch cmd','Pitch')
title('Pitch command and response')
saveas(gcf,'Pitch command and response.png')

figure()
plot(out.time_s,out.de_deg)
xlabel('Time (s)');
title('Elevator')
saveas(gcf,'Elevator.png')

figure()
plot(out.time_s,out.throttle)
xlabel('Time (s)');
title('Throttle')
saveas(gcf,'Throttle.png')

figure()
plot(out.time_s,out.course_cmd_deg)
hold on
plot(out.time_s,out.course_deg)
hold on
xlabel('Time (s)');
legend('Course cmd','Course')
title('Course command and response')
saveas(gcf,'Course command and response.png')

figure()
plot(out.time_s,out.roll_cmd_deg)
hold on
plot(out.time_s,out.roll_deg)
hold on
xlabel('Time (s)');
legend('Roll cmd','Roll')
title('Roll command and response')
saveas(gcf,'Roll command and response.png')

figure()
plot(out.time_s,out.da_deg)
xlabel('Time (s)');
title('Aileron')
saveas(gcf,'Aileron.png')