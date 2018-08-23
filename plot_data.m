%==========================================================================
% ◎ シミュレーションデータプロット
% ※ Drone_System(Drone_Simulation)を実行し、データを得ていることを前提とする。
%==========================================================================
close all;
%--------------------------------------------------------------------------
% ● シミュレーションを実行するときはコメントアウト
%--------------------------------------------------------------------------
% sim('block_nonlinear')
% sim('block_ss')
%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
fig1 = figure(1);
movegui('northwest')
hold on


plot(out.phi/(2*pi)*360,'LineWidth',2);
plot(Lout.phi/(2*pi)*360,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('phi[rad/s]','FontName','arial','FontSize',16)
ylim manual;
legend('Nonlinear','Linear')
set(legend,'FontName','arial','FontSize',14)

fig2 = figure(2);
iptwindowalign(fig1,'right',fig2,'left');
hold on
plot(out.y,'LineWidth',2);
plot(Lout.y,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('速度[m/s]','FontName','arial','FontSize',16)
ylim manual%([-100 0]);
legend('Nonlinear','Linear')
set(legend,'FontName','arial','FontSize',16)

