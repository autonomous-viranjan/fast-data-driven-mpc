% Closed loop fmpc results
close all
figure(1)
%plot(vHostGaussianIn(1:401),'--')
hold on
plot(vEgoFmpc)
%legend('Gaussian Solution','Fast MPC Solution')
title('Velocity')
axis([0 400 0 20])
figure(2)
%plot(sHostGaussianIn(1:401),'--')
hold on
plot(sEgoFmpc)
plot(sLead(1:length(time),1),'-k')
%legend('Gaussian Solution','Fast MPC Solution','Lead Vehicle')
title('Position')

figure(3)
%plot(uHostGaussianIn(1:400),'--')
hold on
plot(uEgoFmpc)
%legend('Gaussian Solution','Fast MPC Solution')
title('Control')