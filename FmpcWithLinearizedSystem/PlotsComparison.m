% Plots multiple Traffic Lights

for i = 1:NumberTrafficLights
    timeTraffic(i,1) = time((i-1)*(tf-1) + tf,1);
end

h = figure(1);
for i = 1:NumberTrafficLights
    plot([timeTraffic(i) ; timeTraffic(i) + 10],...
        [RedLights(i); RedLights(i)], 'r', 'LineWidth', 2)
    hold on
end
for i = 1:NumberTrafficLights
    plot([timeTraffic(i) - 5; timeTraffic(i) + 5],...
        [RedLights(i) + RedLights(1) ; RedLights(i) + RedLights(1)],...
        'r', 'LineWidth', 2)
    hold on
end
for i = 1:NumberTrafficLights
    plot([timeTraffic(i) + 5 ; timeTraffic(i) + 15],...
        [RedLights(i) - RedLights(1) ; RedLights(i) - RedLights(1)],...
        'r', 'LineWidth', 2)
    hold on
end
hold on
h1 = plot(time, sLead(1:length(time),1), 'k--', 'LineWidth', 1.5);
hold on
h2 = plot(time, sHostOptimal, 'k', 'LineWidth', 1.5);
hold on
h3 = plot(time, sHostRandom, 'r', 'LineWidth', 1.5);
hold on
h4 = plot(time, sHostGaussian, 'b', 'LineWidth', 1.5);
grid on
set(gca,'FontName','Times','FontSize',14);
ylim([2000 2700])
xlim([200 260])
xlabel('Time [sec]', 'FontName', 'Times', 'FontSize', 16)
ylabel('Distance [m]', 'FontName', 'Times', 'FontSize', 16)
legend([h1 h2 h3 h4],{'Lead Car','MPC','Random','Gaussian'}, 'Location', 'northwest')

set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

print(h,'MultipleTrafficPositionOptimalRandomGaussian','-dpdf','-r0')

%%
% Comparison of different methods = driving cycles
h = figure(2);
plot(time, vHostOptimal, 'k', 'LineWidth', 1.5)
hold on
plot(time, vHostRandom, 'r', 'LineWidth', 1.5)
hold on
plot(time, vHostGaussian, 'b', 'LineWidth', 1.5)
ylim([0 20])
grid on
legend('MPC','Random','Gaussian', 'Location', 'northwest')
set(gca,'FontName','Times','FontSize',14);
xlabel('Time [sec]', 'FontName', 'Times', 'FontSize', 16)
ylabel('Speed [m/s]', 'FontName', 'Times', 'FontSize', 16)

set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

print(h,'MultipleTrafficSpeedOptimalRandomGaussian','-dpdf','-r0')


%%

figure(3)
plot(time(1:length(time)-1,1), uHostOptimal, 'k', 'LineWidth', 1.5)
hold on
plot(time(1:length(time)-1,1), uHostRandom, 'r', 'LineWidth', 1.5)
hold on
plot(time(1:length(time)-1,1), uHostGaussian, 'b', 'LineWidth', 1.5)
ylim([uMin uMax])
grid on
legend('MPC','Random','Gaussian')
set(gca,'FontName','Times','FontSize',14);
xlabel('Time [sec]', 'FontName', 'Times', 'FontSize', 14)
ylabel('u [m/s^2]', 'FontName', 'Times', 'FontSize', 14)

%%

x = linspace(0,10,10000);
beta1 = 0.6; beta2 = 1.4;
sigma = beta1./abs(x - beta2);
x1 = beta2*ones(1,10000);
asymptote = linspace(0,100,10000);

h = figure(4);
h1 = plot(x,sigma,'k','LineWidth', 2);
hold on
plot(x1,asymptote,'k--','LineWidth', 2)
beta1 = 0.2; beta2 = 1.4;
sigma = beta1./abs(x - beta2);
x1 = beta2*ones(1,10000);
asymptote = linspace(0,100,10000);
hold on
h2 = plot(x,sigma,'b','LineWidth', 2);
hold on
plot(x1,asymptote,'k--','LineWidth', 2)
axis([1 1.8 0 10])
grid on
set(gca,'FontName','Times','FontSize',14);
legend([h1 h2],{'\beta_1 = 0.6','\beta_1 = 0.2'}, 'Location', 'northeast', 'FontSize', 14)
xlabel('${\Delta s_a}/{\Delta s_c}$','Interpreter','latex','FontSize', 22)
ylabel('$\sigma$','Interpreter','latex','FontSize', 22)

set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

print(h,'StandardDeviation','-dpdf','-r0')

