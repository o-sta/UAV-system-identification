%==========================================================================
% ◎ ドローンシステム同定(2段階システム同定)
% 入力 desire_phi desire_theta
% 出力 x y u v
% step1で入力を ft tx ty tz とし、出力を z w phi theta psi p q r である
% 不安定システムの同定を行い、安定化コントローラを設計した後に行う。
% 入力を目標値 phi theta または z phi theta psi または phi theta psi
% 出力を x y または u v または x y u v
% である開ループシステム同定を行う
%==========================================================================
close all;
clear;
addpath('../SI_Function');
%--------------------------------------------------------------------------
% ● データの読み込み
%--------------------------------------------------------------------------
load data/drone;                             % ドローンの定数(Ix Iy Iz m g)
load data/regulator_gain_of_attitude;   % 姿勢に関する最適レギュレータのゲイン

%--------------------------------------------------------------------------
% ● パラメータ設定
% 最初の10秒(start_time)はシステムの収束のために使用され、次の10秒間(length_of_time)で
% システム同定に必要なデータを取得します。
%--------------------------------------------------------------------------
identification_mode = 'y';              % 何の状態を同定するか('x' or 'y' or 'xy')
estimated_matrix_save = true;           % 同定した行列を保存する場合はtrue
number_of_state_variables = 3;          % 状態変数の数
number_of_rows_in_data_Matrix = 100;    % データ行列の行数
continuous_signal_ts = 0.001;           % 連続時間シミュレーションのサンプリング周期
                                        % (M系列連続信号のサンプリング周期)
simulation_sampling_ts = 0.001;         % 離散時間信号のサンプリング周期
start_time = 10;                        % 同定開始時間（M系列信号入力開始時間）
length_of_time = 30;                    % 同定に使用する時間
end_time = start_time + length_of_time; % 同定終了時間（シミュレーション終了時間）
min_value = [2*pi/180 2*pi/180];        % M系列信号の最小値 [phi theta]
max_value = -min_value;                 % M系列信号の最大値 [phi theta]
MLS_degree = 7;                         % M系列信号の次数(2 3 4 5 6 7 9 10 11 15)
MLS_seeds = [7 4];                      % M系列信号のシード値
MLS_seeds_compare = [9 10];             % M系列信号のシード値（比較用）
MLS_degree_compare = 5;                 % M系列信号の次数(2 3 4 5 6 7 9 10 11 15)

%--------------------------------------------------------------------------
% ● M系列信号作成
%--------------------------------------------------------------------------
% [continuous_signal, discrete_signal, ts] = 
% createMLS(number, seeds, min_value, max_value, length_of_time, continuous_signal_ts)
% continuous_signal     二値連続時間信号
% discrete_signal       二値離散時間信号
% ts                    二値離散時間信号のサンプリング周期
% number                number段シフトレジスタ（デフォルトは5）
% seeds                 シード値 seeds mod 2^number を2進数で表したものがレジスタの状態xの初期値になるが、
%                       0になったときはランダムにシードが選択される
% min_value             二値信号の最小値
% max                   二値信号の最大値
% length_of_time        信号全体の時間
% continuous_signal_ts  連続時間信号に格納するデータのサンプリング時間
%--------------------------------------------------------------------------
mls_phi = mf_createMLS(MLS_degree, MLS_seeds(1), min_value(1), max_value(1), length_of_time, continuous_signal_ts);
mls_theta = mf_createMLS(MLS_degree, MLS_seeds(2), min_value(2), max_value(2), length_of_time, continuous_signal_ts);

%--------------------------------------------------------------------------
% ● シミュレーション及びMOESP実行
%--------------------------------------------------------------------------
sim('block/xy_sysid', end_time);
switch identification_mode
case 'x'
    data_discrete = getsampleusingtime(data_discrete, start_time, end_time);
    input_data = data_discrete.data(:, 25)';     % 入力データ行列
    output_data = horzcat(data_discrete.data(:, 1), data_discrete.data(:, 4))'; % 出力データ行列
case 'y'
    data_discrete = getsampleusingtime(data_discrete, start_time, end_time);
    input_data = data_discrete.data(:, 24)';     % 入力データ行列
    output_data = horzcat(data_discrete.data(:, 2), data_discrete.data(:, 5))'; % 出力データ行列
case 'xy'
    data_discrete = getsampleusingtime(data_discrete, start_time, end_time);
    input_data = data_discrete.data(:, 24:25)';     % 入力データ行列
    output_data = horzcat(data_discrete.data(:, 1:2), data_discrete.data(:, 4:5))'; % 出力データ行列
end

% est_d_A : estimated discrete matrix A
[est_d_A, est_d_B, est_d_C, est_d_D] = mf_moesp(input_data, output_data, number_of_rows_in_data_Matrix, number_of_state_variables);

%--------------------------------------------------------------------------
% ● データ比較
%--------------------------------------------------------------------------
% M系列信号を同定時とは違うものにする
mls_phi = mf_createMLS(MLS_degree_compare, MLS_seeds_compare(1), min_value(1), max_value(1), length_of_time, continuous_signal_ts);
mls_theta = mf_createMLS(MLS_degree_compare, MLS_seeds_compare(2), min_value(2), max_value(2), length_of_time, continuous_signal_ts);
% 連続時間状態方程式の作成
estimated_discrete_system = ss(est_d_A, est_d_B, est_d_C, est_d_D, simulation_sampling_ts);
estimated_continuous_system = d2c(estimated_discrete_system, 'tustin');
[est_c_A, est_c_B, est_c_C, est_c_D] = ssdata(estimated_continuous_system); % est_c_A : estimated continuous matrix A
matrix_A = est_c_A;
matrix_B = est_c_B;
matrix_C = est_c_C;
matrix_D = est_c_D;
sim('block/xy_compare', end_time);
data_est = getsampleusingtime(data_est, start_time, end_time);

%--------------------------------------------------------------------------
% ● FIT算出
% mf_fit関数を用いて算出する。
%--------------------------------------------------------------------------
switch identification_mode
case 'x'
    data_est_discrete = getsampleusingtime(data_est_discrete, start_time, end_time);
    fit_percent = [mf_fit(data_est_discrete.Data(:,1), data_est_discrete.Data(:,27)) 0 0;
    mf_fit(data_est_discrete.Data(:,4), data_est_discrete.Data(:,29)) 0 0;
    0 0 0;
    0 0 0;]
case 'y'
    data_est_discrete = getsampleusingtime(data_est_discrete, start_time, end_time);
    fit_percent = [0 mf_fit(data_est_discrete.Data(:,2), data_est_discrete.Data(:,28)) 0;
    0 mf_fit(data_est_discrete.Data(:,5), data_est_discrete.Data(:,30)) 0;
    0 0 0;
    0 0 0;]
case 'xy'
    data_est_discrete = getsampleusingtime(data_est_discrete, start_time, end_time);
    fit_percent = [mf_fit(data_est_discrete.Data(:,1), data_est_discrete.Data(:,27)) mf_fit(data_est_discrete.Data(:,2), data_est_discrete.Data(:,28)) 0;
    mf_fit(data_est_discrete.Data(:,4), data_est_discrete.Data(:,29)) mf_fit(data_est_discrete.Data(:,5), data_est_discrete.Data(:,30)) 0;
    0 0 0;
    0 0 0;]
end

%--------------------------------------------------------------------------
% ● 行列の保存
%--------------------------------------------------------------------------
if estimated_matrix_save
    switch identification_mode
    case 'x'
        save('data/estimated_matrix_x','matrix_A','matrix_B','matrix_C','matrix_D');
    case 'y'
        save('data/estimated_matrix_y','matrix_A','matrix_B','matrix_C','matrix_D');
    case 'xy'
        save('data/estimated_matrix_xy','matrix_A','matrix_B','matrix_C','matrix_D');
    end
end

%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
switch identification_mode
case 'x'
    fig1 = figure(1);
    fig1.Position = [0 0 800 800];
    movegui('northwest')
    subplot(4,3,1)
    plot(data_est.time,data_est.data(:,1),'LineWidth',1.5) % x 1,1
    hold on
    plot(data_est.time,data_est.data(:,27),'--','LineWidth',1.5) % x 1,1
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
    plot(data_est.time,data_est.data(:,29),'--','LineWidth',1.5) % x 1,1
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
    
    fig2 = figure(2);
    fig2.Position = [0 0 400 800];
    iptwindowalign(fig1,'right',fig2,'left');
    % subplot(4,1,1)
    % plot(data_est.time,data_est.data(:,23),'LineWidth',1.5) % target_z 1
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_z [m]','FontName','arial','FontSize',10)
    % subplot(4,1,2)
    % plot(data_est.time,data_est.data(:,24)*180/pi,'LineWidth',1.5) % target_phi 2
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_phi [deg]','FontName','arial','FontSize',10)
    subplot(4,1,3)
    plot(data_est.time,data_est.data(:,25)*180/pi,'LineWidth',1.5) % target_theta 3
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('target_theta [deg]','FontName','arial','FontSize',10)
    % subplot(4,1,4)
    % plot(data_est.time,data_est.data(:,26),'LineWidth',1.5) % target_psi 4
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_psi [deg]','FontName','arial','FontSize',10)
case 'y'
    fig1 = figure(1);
    fig1.Position = [0 0 800 800];
    movegui('northwest')
    subplot(4,3,1)
    plot(data_est.time,data_est.data(:,1),'LineWidth',1.5) % x 1,1
    hold on
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('x [m]','FontName','arial','FontSize',10)
    subplot(4,3,2)
    plot(data_est.time,data_est.data(:,2),'LineWidth',1.5) % y 1,2
    hold on
    plot(data_est.time,data_est.data(:,28),'--','LineWidth',1.5) % y 1,1
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
    plot(data_est.time,data_est.data(:,30),'--','LineWidth',1.5) % y 1,1
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
    
    fig2 = figure(2);
    fig2.Position = [0 0 400 800];
    iptwindowalign(fig1,'right',fig2,'left');
    % subplot(4,1,1)
    % plot(data_est.time,data_est.data(:,23),'LineWidth',1.5) % target_z 1
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_z [m]','FontName','arial','FontSize',10)
    subplot(4,1,2)
    plot(data_est.time,data_est.data(:,24)*180/pi,'LineWidth',1.5) % target_phi 2
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('target_phi [deg]','FontName','arial','FontSize',10)
    % subplot(4,1,3)
    % plot(data_est.time,data_est.data(:,25)*180/pi,'LineWidth',1.5) % target_theta 3
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_theta [deg]','FontName','arial','FontSize',10)
    % subplot(4,1,4)
    % plot(data_est.time,data_est.data(:,26),'LineWidth',1.5) % target_psi 4
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_psi [deg]','FontName','arial','FontSize',10)
case 'xy'
    fig1 = figure(1);
    fig1.Position = [0 0 800 800];
    movegui('northwest')
    subplot(4,3,1)
    plot(data_est.time,data_est.data(:,1),'LineWidth',1.5) % x 1,1
    hold on
    plot(data_est.time,data_est.data(:,27),'--','LineWidth',1.5) % x 1,1
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('x [m]','FontName','arial','FontSize',10)
    subplot(4,3,2)
    plot(data_est.time,data_est.data(:,2),'LineWidth',1.5) % y 1,2
    hold on
    plot(data_est.time,data_est.data(:,28),'--','LineWidth',1.5) % x 1,1
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
    plot(data_est.time,data_est.data(:,29),'--','LineWidth',1.5) % x 1,1
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('u [m/s]','FontName','arial','FontSize',10)
    subplot(4,3,5)
    plot(data_est.time,data_est.data(:,5),'LineWidth',1.5) % v 2,2
    hold on
    plot(data_est.time,data_est.data(:,30),'--','LineWidth',1.5) % x 1,1
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
    
    fig2 = figure(2);
    fig2.Position = [0 0 400 800];
    iptwindowalign(fig1,'right',fig2,'left');
    % subplot(4,1,1)
    % plot(data_est.time,data_est.data(:,23),'LineWidth',1.5) % target_z 1
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_z [m]','FontName','arial','FontSize',10)
    subplot(4,1,2)
    plot(data_est.time,data_est.data(:,24)*180/pi,'LineWidth',1.5) % target_phi 2
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('target_phi [deg]','FontName','arial','FontSize',10)
    subplot(4,1,3)
    plot(data_est.time,data_est.data(:,25)*180/pi,'LineWidth',1.5) % target_theta 3
    xlabel('time [s]','FontName','arial','FontSize',10)
    ylabel('target_theta [deg]','FontName','arial','FontSize',10)
    % subplot(4,1,4)
    % plot(data_est.time,data_est.data(:,26),'LineWidth',1.5) % target_psi 4
    % xlabel('time [s]','FontName','arial','FontSize',10)
    % ylabel('target_psi [deg]','FontName','arial','FontSize',10)
end
