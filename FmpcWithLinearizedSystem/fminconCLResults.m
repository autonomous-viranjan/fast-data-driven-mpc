% Closed loop fmincon results
close all
figure(1)
plot(vHostGaussianIn(1:401),'--')
hold on
plot(vHostTotal)
legend('Gaussian Solution','fmincon MPC Solution')
title('Velocity')

figure(2)
plot(sHostGaussianIn(1:401),'--')
hold on
plot(sHostTotal)
plot(sLead(1:length(time),1),'-k')
legend('Gaussian Solution','fmincon MPC Solution','Lead Vehicle')
title('Position')

figure(3)
plot(uHostGaussianIn(1:400),'--')
hold on
plot(uHostTotal)
legend('Gaussian Solution','fmincon MPC Solution')
title('Control')