% Plots multiple Traffic Lights

for i = 1:NumberTrafficLights
    timeTraffic(i,1) = time((i-1)*(tf-1) + tf,1);
end

h = figure(2);
for i = 1:NumberTrafficLights
    plot([timeTraffic(i) ; timeTraffic(i) + 10],...
        [RedLights(i); RedLights(i)], 'r', 'LineWidth', 3)
    hold on
end
for i = 1:NumberTrafficLights
    plot([timeTraffic(i) - 5; timeTraffic(i) + 5],...
        [RedLights(i) + RedLights(1) ; RedLights(i) + RedLights(1)],...
        'r', 'LineWidth', 3)
    hold on
end
for i = 1:NumberTrafficLights
    plot([timeTraffic(i) + 5 ; timeTraffic(i) + 15],...
        [RedLights(i) - RedLights(1) ; RedLights(i) - RedLights(1)],...
        'r', 'LineWidth', 3)
    hold on
end
hold on
h1 = plot(time, sLead(1:length(time),1), 'k--', 'LineWidth', 1.5);
hold on
h2 = plot(time(1:length(sHostTotal)), sHostTotal, 'k', 'LineWidth', 1.5);
grid on
set(gca,'FontName','Times','FontSize',12);
xlim([0 tfsecF])
xlabel('Time [sec]', 'FontName', 'Times', 'FontSize', 14)
ylabel('Distance [m]', 'FontName', 'Times', 'FontSize', 14)
legend([h1 h2],{'Lead Car','Host Car'}, 'Location', 'northwest')

set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

print(h,'MultipleTrafficPositionOptimal','-dpdf','-r0')


%%
sConst = dMin + alpha*vHostTotal;
sActual = sLead(1:length(time),1) - sHostTotal;

h = figure(2);
subplot(2,1,1)
plot(time, sConst, 'r', 'LineWidth', 1.5);
hold on
plot(time, sActual, 'k', 'LineWidth', 1.5);
set(gca,'FontName','Times','FontSize',12);
ylabel('s_{Lead} - s_{Host} [m]', 'FontName', 'Times', 'FontSize', 14)
grid on
legend('Const', 'Actual','Location', 'northwest')

subplot(2,1,2)
plot(time, vHostTotal, 'b', 'LineWidth', 1.5);
hold on
plot(time, vLead(1:length(time),1), 'k', 'LineWidth', 1.5);
set(gca,'FontName','Times','FontSize',12);
xlabel('Time [sec]', 'FontName', 'Times', 'FontSize', 14)
ylabel('v [m/s]', 'FontName', 'Times', 'FontSize', 14)
ylim([0 20])
grid on

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% 
% print(h,'MultipleTrafficSpeed','-dpdf','-r0')

%%
h = figure(5);
plot(time(1:length(time)-1,1), uHostTotal, 'b', 'LineWidth', 1.5); 
hold on
plot(time(1:length(time)-1,1), uLead(1:length(time)-1,1), 'k', 'LineWidth', 1.5);
set(gca,'FontName','Times','FontSize',12);
xlabel('Time [sec]', 'FontName', 'Times', 'FontSize', 14)
ylabel('u [m/s^2]', 'FontName', 'Times', 'FontSize', 14)
grid on
legend('Host Car','Lead Car', 'Location', 'northwest')
set(h,'Units','Inches');
pos = get(h,'Position');
ylim([uMin uMax])
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% 
% print(h,'MultipleTrafficControl','-dpdf','-r0')

%%
% h = figure(3);
% plot(time, vLead, 'k', 'LineWidth', 1.5);
% hold on
% plot(time, vHostTotal, 'b', 'LineWidth', 1.5);

%%
h = figure(4);
plot(time, vLead - vHostTotal, 'k', 'LineWidth', 1.5);

