%==========================================================================
% ◎ シミュレーションデータプロット
% ※ Drone_System(Drone_Simulation)を実行し、データを得ていることを前提とする。
%==========================================================================
close all;
%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
fig1 = figure(1);
movegui('northwest')
hold on
plot(data.Time,data.Data(:,7)/(2*pi)*360,'LineWidth',2);
plot(mo_t,mo_y/(2*pi)*360,'--','LineWidth',2);
plot(n4_t,n4_y/(2*pi)*360,'-.','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('phi [deg]','FontName','arial','FontSize',16)
ylim manual;
% legend('Simulation','MOESP','N4SID')
% set(legend,'FontName','arial','FontSize',14)

fig2 = figure(2);
iptwindowalign(fig1,'right',fig2,'left');
hold on
plot(pdt_mt, pdt_true*180/pi,'LineWidth',2);
plot(pdt_mt, pdt_m*180/pi,'--','LineWidth',2);
plot(pdt_nt, pdt_n*180/pi,'-.','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('phi [deg]','FontName','arial','FontSize',16)
legend('True','moesp','n4sid')
set(legend,'FontName','arial','FontSize',16,'Location','southeast')


% M系列信号と自己相関関数
fig3 = figure(3);
iptwindowalign(fig2,'right',fig3,'left');
subplot(2,1,1);
plot(data.Time,data.Data(:,14),'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('torque[Nm]','FontName','arial','FontSize',16)
subplot(2,1,2);
stem(xcorr(u))


