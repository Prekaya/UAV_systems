
figure()
plot(out.time_s,out.pitch_deg)
hold on
plot(out.time_s,out.pitch_est_deg)
hold on
legend('Pitch\_deg','Pitch\_est\_deg')
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Pitch Compare');

