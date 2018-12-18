%==========================================================================
% ◎ ドローンの低レベルコントローラ設計とシミュレーション
% 姿勢制御に関するドローンのオブザーバと最適レギュレータのゲインを設計し、
% シミュレーションを行います。
% 入力 tx ty tz
% 出力 phi theta psi p q r
%==========================================================================
close all;
clear;
addpath('../SI_Function');
load data/drone; % ドローンのデータを読み込む m g Ix Iy Iz

%--------------------------------------------------------------------------
% ● システム同定で推定した行列を読み込む
%--------------------------------------------------------------------------
load data/estimated_matrix_x; % matrix_A, matrix_B, matrix_C, matrix_D を読み込む
                                    %それぞれ状態空間表現の行列A,B,C,Dである
matrix_xA = matrix_A;
matrix_xB = matrix_B;
matrix_xC = matrix_C;
matrix_xD = matrix_D;
load data/estimated_matrix_y; % matrix_A, matrix_B, matrix_C, matrix_D を読み込む
                                    %それぞれ状態空間表現の行列A,B,C,Dである
matrix_yA = matrix_A;
matrix_yB = matrix_B;
matrix_yC = matrix_C;
matrix_yD = matrix_D;
load data/regulator_gain_of_attitude

%--------------------------------------------------------------------------
% ● 目標値制限設定
%--------------------------------------------------------------------------
regulated_target_angular = 10*pi/180; %目標姿勢 phi theta の上限
regulated_target_velocity = 1;

%--------------------------------------------------------------------------
% ● PIDコントローラゲイン設定
%--------------------------------------------------------------------------
vp = 4;
vi = 3;
vd = 0.5;
xp = 1;

yvp = -4;
yvi = -3;
yvd = -0.5;
yp = 1;

%--------------------------------------------------------------------------
% ● ターゲット設定
%--------------------------------------------------------------------------
xt = 0;
yt = -10;
zt = 0;
psit = 90*pi/180;

%--------------------------------------------------------------------------
% ● シミュレーション
%--------------------------------------------------------------------------
sim('block/xy_control',100);

%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
fig1 = figure(1);
fig1.Position = [0 0 800 800];
subplot(4,3,1)
plot(data_est.time,data_est.data(:,1),'LineWidth',1.5) % x 1,1
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('x [m]','FontName','arial','FontSize',10)
subplot(4,3,2)
plot(data_est.time,data_est.data(:,2),'LineWidth',1.5) % y 1,2
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('y [m]','FontName','arial','FontSize',10)
subplot(4,3,3)
plot(data_est.time,data_est.data(:,3),'LineWidth',1.5) % z 1,3
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('z [m]','FontName','arial','FontSize',10)

subplot(4,3,4)
plot(data_est.time,data_est.data(:,4),'LineWidth',1.5) % u 2,1
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('u [m/s]','FontName','arial','FontSize',10)
subplot(4,3,5)
plot(data_est.time,data_est.data(:,5),'LineWidth',1.5) % v 2,2
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('v [m/s]','FontName','arial','FontSize',10)
subplot(4,3,6)
plot(data_est.time,data_est.data(:,6),'LineWidth',1.5) % w 2,3
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('w [m/s]','FontName','arial','FontSize',10)

subplot(4,3,7)
plot(data_est.time,data_est.data(:,7)*180/pi,'LineWidth',1.5) % phi 3,1
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('phi [deg]','FontName','arial','FontSize',10)
subplot(4,3,8)
plot(data_est.time,data_est.data(:,8)*180/pi,'LineWidth',1.5) % theta 3,2
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('theta [deg]','FontName','arial','FontSize',10)
subplot(4,3,9)
plot(data_est.time,data_est.data(:,9)*180/pi,'LineWidth',1.5) % psi 3,3
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('psi [deg]','FontName','arial','FontSize',10)

subplot(4,3,10)
plot(data_est.time,data_est.data(:,10)*180/pi,'LineWidth',1.5) % p 4,1
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('p [deg/s]','FontName','arial','FontSize',10)
subplot(4,3,11)
plot(data_est.time,data_est.data(:,11)*180/pi,'LineWidth',1.5) % q 4,2
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('q [deg/s]','FontName','arial','FontSize',10)
subplot(4,3,12)
plot(data_est.time,data_est.data(:,12)*180/pi,'LineWidth',1.5) % r 4,3
hold on
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
ylabel('tx [Nm]','FontName','arial','FontSize',10)
subplot(4,1,3)
plot(data_est.time,data_est.data(:,15),'LineWidth',1.5) % fy 3
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('ty [Nm]','FontName','arial','FontSize',10)
subplot(4,1,4)
plot(data_est.time,data_est.data(:,16),'LineWidth',1.5) % fz 4
hold on
xlabel('time [s]','FontName','arial','FontSize',10)
ylabel('tz [Nm]','FontName','arial','FontSize',10)
iptwindowalign(fig1,'right',fig2,'left');


