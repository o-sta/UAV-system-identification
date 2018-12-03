%==========================================================================
% ◎ ドローンの低レベルコントローラ設計とシミュレーション
% 姿勢制御に関するドローンのオブザーバと最適レギュレータのゲインを設計し、
% シミュレーションを行います。
%==========================================================================
close all;
clear;
addpath('SI_Function');
%--------------------------------------------------------------------------
% ● 状態空間表現の定義
%--------------------------------------------------------------------------
syms Ix Iy Iz m g                 % 慣性モーメントと質量、重力加速度
load data/drone_ss;
load data/drone_data;
v_syms = [Ix Iy Iz m g];
v_vals = [drone_Ix drone_Iy drone_Iz drone_m drone_g];

%--------------------------------------------------------------------------
% ● 値の代入
%--------------------------------------------------------------------------
for i = 1:numel(v_syms)
  A = subs(A, v_syms(i), v_vals(i));
  B = subs(B, v_syms(i), v_vals(i));
  D = subs(D, v_syms(i), v_vals(i));
end
clear Ix Iy Iz m g;
Ix = v_vals(1);
Iy = v_vals(2);
Iz = v_vals(3);
m = v_vals(4);
g = v_vals(5);
c = 8.048*10^(-6);
l = 0.1785;
q = 2.423*19^(-7);

A_val = double(A);
B_val = double(B);
D_val = double(D);

%--------------------------------------------------------------------------
% ● システム同定で推定した行列を読み込む
%--------------------------------------------------------------------------
load data/estimated_matrix; % matrix_A, matrix_B, matrix_C, matrix_D を読み込む
                            %それぞれ状態空間表現の行列A,B,C,Dである

%--------------------------------------------------------------------------
% ● 最適レギュレータゲインの設計
% 出力と入力に重みをつける
%--------------------------------------------------------------------------
weight_U = diag([1 1 1 1]);         % 入力に関する重み [ft tx ty tz]
weight_Y = diag([1 1 1 1 1 1 1 1]); % 出力に関する重み [z w phi theta psi p q r]
% 上記で定義した入出力の重みから
% 状態の重みQ、入力の重みR、状態と入力の重みNを求める
weight_Q = matrix_C' * weight_Y * matrix_C;
weight_R = weight_U + matrix_D' * weight_Y * matrix_D;
weight_N = matrix_C' * weight_Y * matrix_D;
% reg_K : 最適レギュレータのゲイン
% reg_S : リカッチ方程式の解
% reg_eigenvalue : レギュレータ実装時のシステムの極(A-BKの固有値)
[reg_K, reg_S, reg_eigenvalue] = lqr(matrix_A, matrix_B, weight_Q, weight_R, weight_N);
% [reg_K, reg_S, reg_eigenvalue] = lqr(matrix_A, matrix_B, weight_Y, weight_U);
%--------------------------------------------------------------------------
% ● オブザーバゲインの設計
%--------------------------------------------------------------------------
obs_L = place(matrix_A',matrix_C',reg_eigenvalue*1.5).';

%--------------------------------------------------------------------------
% ● シミュレーション
%--------------------------------------------------------------------------
sim('drone_nonlinear_full_control',20);

%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
fig1 = figure(1);
fig1.Position = [0 0 800 800];
% subplot(4,3,1)
% plot(data_est.time,data_est.data(:,1),'LineWidth',1.5) % x 1,1
% hold on
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('x [m]','FontName','arial','FontSize',10)
% subplot(4,3,2)
% plot(data_est.time,data_est.data(:,2),'LineWidth',1.5) % y 1,2
% hold on
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('y [m]','FontName','arial','FontSize',10)
subplot(4,3,3)
plot(data_est.time,data_est.data(:,3),'LineWidth',1.5) % z 1,3
hold on
plot(data_est.time,data_est.data(:,27),'LineWidth',1.5) % z 1,3
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('z [m]','FontName','arial','FontSize',10)

% subplot(4,3,4)
% plot(data_est.time,data_est.data(:,4),'LineWidth',1.5) % u 2,1
% hold on
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('u [m/s]','FontName','arial','FontSize',10)
% subplot(4,3,5)
% plot(data_est.time,data_est.data(:,5),'LineWidth',1.5) % v 2,2
% hold on
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('v [m/s]','FontName','arial','FontSize',10)
subplot(4,3,6)
plot(data_est.time,data_est.data(:,6),'LineWidth',1.5) % w 2,3
hold on
plot(data_est.time,data_est.data(:,28),'LineWidth',1.5) % w 2,3
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('w [m/s]','FontName','arial','FontSize',10)

subplot(4,3,7)
plot(data_est.time,data_est.data(:,7)*180/pi,'LineWidth',1.5) % phi 3,1
hold on
plot(data_est.time,data_est.data(:,29)*180/pi,'LineWidth',1.5) % phi 3,1
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('phi [deg]','FontName','arial','FontSize',10)
subplot(4,3,8)
plot(data_est.time,data_est.data(:,8)*180/pi,'LineWidth',1.5) % theta 3,2
hold on
plot(data_est.time,data_est.data(:,30)*180/pi,'LineWidth',1.5) % theta 3,2
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('theta [deg]','FontName','arial','FontSize',10)
subplot(4,3,9)
plot(data_est.time,data_est.data(:,9)*180/pi,'LineWidth',1.5) % psi 3,3
hold on
plot(data_est.time,data_est.data(:,31)*180/pi,'LineWidth',1.5) % psi 3,3
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('psi [deg]','FontName','arial','FontSize',10)

subplot(4,3,10)
plot(data_est.time,data_est.data(:,10)*180/pi,'LineWidth',1.5) % p 4,1
hold on
plot(data_est.time,data_est.data(:,32)*180/pi,'LineWidth',1.5) % p 4,1
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('p [deg/s]','FontName','arial','FontSize',10)
subplot(4,3,11)
plot(data_est.time,data_est.data(:,11)*180/pi,'LineWidth',1.5) % q 4,2
hold on
plot(data_est.time,data_est.data(:,33)*180/pi,'LineWidth',1.5) % q 4,2
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('q [deg/s]','FontName','arial','FontSize',10)
subplot(4,3,12)
plot(data_est.time,data_est.data(:,12)*180/pi,'LineWidth',1.5) % r 4,3
hold on
plot(data_est.time,data_est.data(:,34)*180/pi,'LineWidth',1.5) % r 4,3
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('r [deg/s]','FontName','arial','FontSize',10)
movegui('northwest')

fig2 = figure(2);
fig2.Position = [0 0 400 800];
subplot(4,1,1)
plot(data_est.time,data_est.data(:,13),'LineWidth',1.5) % ft 1
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('ft [N]','FontName','arial','FontSize',10)
subplot(4,1,2)
plot(data_est.time,data_est.data(:,14),'LineWidth',1.5) % fx 2
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('tx [N]','FontName','arial','FontSize',10)
subplot(4,1,3)
plot(data_est.time,data_est.data(:,15),'LineWidth',1.5) % fy 3
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('ty [N]','FontName','arial','FontSize',10)
subplot(4,1,4)
plot(data_est.time,data_est.data(:,16),'LineWidth',1.5) % fz 4
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('tz [N]','FontName','arial','FontSize',10)
iptwindowalign(fig1,'right',fig2,'left');

% fig3 = figure(3);
% fig3.Position = [0 0 400 800];
% subplot(4,1,1)
% plot(data_est.time,data_est.data(:,27),'LineWidth',1.5) % desire_z 1
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('signal ft [N]','FontName','arial','FontSize',10)
% subplot(4,1,2)
% plot(data_est.time,data_est.data(:,28),'LineWidth',1.5) % desire_phi 2
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('signal tx [Nm]','FontName','arial','FontSize',10)
% subplot(4,1,3)
% plot(data_est.time,data_est.data(:,29),'LineWidth',1.5) % desire_theta 3
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('signal ty [Nm]','FontName','arial','FontSize',10)
% subplot(4,1,4)
% plot(data_est.time,data_est.data(:,30),'LineWidth',1.5) % desire_psi 4
% xlabel('time [s]','FontName','arial','FontSize',10)
% ylabel('signal tz [Nm]','FontName','arial','FontSize',10)
% iptwindowalign(fig2,'right',fig3,'left');

