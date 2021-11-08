close all
% Closed loop fmpc results
figure(1)
%plot(vHostGaussianIn,'--')
%hold on
plot(vEgoTotaldmd)
%legend('Gaussian Solution','Fast MPC Solution')
title('Velocity')
axis([0 400 0 20])
figure(2)
plot(sLead(1:401),'--')
hold on
plot(sEgoTotaldmd)
%legend('Gaussian Solution','Fast MPC Solution')
title('Position')

figure(3)
%plot(uHostGaussianIn,'--')
%hold on
plot(uEgoTotaldmd)
%legend('Gaussian Solution','Fast MPC Solution')
title('Control')