%==========================================================================
% ◎ システム同定シミュレーション（倒立振子,上向き基準）
%==========================================================================
close all;
clear;
addpath('SI_Function');
cdip_para;
% --------------------------------------------------------------------------
% ● 状態空間表現の定義(振子上向き基準)
% --------------------------------------------------------------------------
tmp = Jp+mp*lp^2;
A = [0 0 1 0;
    0 0 0 1;
    0 0 -ac 0;
    0 mp*g*lp/tmp mp*lp*ac/tmp -mup/tmp];
B = [0;
    0;
    bc;
    -mp*lp*bc/tmp];
C = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
D = [0;
    0;
    0;
    0];
clear tmp;

%--------------------------------------------------------------------------
% ● 最適レギュレータによるゲイン設定
%--------------------------------------------------------------------------
Q = diag([500 2000 1 1]);
R = 30;
K = lqr(A,B,Q,R);

%--------------------------------------------------------------------------
% ● 伝達関数と出力信号の生成
%--------------------------------------------------------------------------
% パラメータ
m = 1;          % 入力の数
p = 2;          % 出力の数
sample = 1;     % サンプリング周波数
n = 4;          % 状態変数の次数
k = 100;        % データ行列の行数
ts = 0.001;      % サンプリング時間

%--------------------------------------------------------------------------
% ● M系列信号生成
%--------------------------------------------------------------------------
% mP = 5;     % M系列のmP段シフトレジスタ
% mk = 2;     % 最終段以外にフィードバックさせる信号
mP = 10;     % M系列のmP段シフトレジスタ
mk = 3;     % 最終段以外にフィードバックさせる信号

x = randi([0 1], mP, 1);    % 状態の初期値をランダムに決定
%x = [0 1 0 0 1]';           % 状態の初期値を指定（全て0は禁止）
u = zeros(2^mP-1, 1);       % 出力変数の定義
for i = 1:2^mP-1            % 信号の生成
    x = [xor(x(mk),x(end));x(1:end-1)]; % フィードバック
    u(i) = x(1);
end
for i = 1:2^mP - 1          % 2値信号の値を0,1から他の値に変更
    if u(i) == 1
        u(i)=1;
    else
        u(i)=-1;
    end
end
s_t = 5;        % 信号全体の時間[s]
s_tp = 0.001;   % 離散信号1個あたりの継続時間[s] / サンプリング時間
s_p = round(s_t/s_tp/numel(u));               % 1個あたりの連続時間信号数
t_data(:,1) = 0:s_tp:s_tp*s_p*numel(u)-s_tp;  % 時間信号を作成
u_data = zeros(size(t_data));
for i = 1:numel(u)  % 信号を作成
    for j = (i-1)*s_p+1:i*s_p
        u_data(j) = u(i);
    end
end
u_in = timeseries(u_data, t_data, 'name', 'Position');  % 時系列作成
sim('block/pen_linear_close',s_t);

%--------------------------------------------------------------------------
% ● CL-MOESP実行
%--------------------------------------------------------------------------
reference_data = data_D.data(:,1)';
input_data = data_D.data(:, 2)';
output_data = data_D.data(:,3:6)';

[clA, clB, clC, clD] = mf_clmoesp(reference_data, input_data, output_data, k, n);

%--------------------------------------------------------------------------
% ● 同定結果の比較 CL-MOESP
%--------------------------------------------------------------------------
dP_est_clmoesp = ss(clA, clB, clC, clD, ts);
cP_est_clmoesp = d2c(dP_est_clmoesp, 'zoh');
[coA, coB, coC, coD] = ssdata(cP_est_clmoesp);
sim('block/pen_linear_close_compare',s_t);

cl_t = Cdata.time;                         % 時間
cl_ref = Cdata.data(:,1);                  % 目標値
cl_int = Cdata.data(:,2);                  % 入力 真値
cl_ini = Cdata.data(:,7);                  % 入力 同定
cl_out1t = Cdata.data(:,3);                % 出力1 真値
cl_out2t = Cdata.data(:,4)/(2*pi)*360;     % 出力2 真値
cl_out1i = Cdata.data(:,8);                % 出力1 同定
cl_out2i = Cdata.data(:,9)/(2*pi)*360;     % 出力2 同定

clfit1 = mf_fit(cl_out1i,cl_out1t)
clfit2 = mf_fit(cl_out2i,cl_out2t)

%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
% 目標値信号と自己相関関数(PE性確認のため)
fig1 = figure(1);
movegui('northwest')
subplot(2,1,1);
hold on
plot(cl_t,cl_ref,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('reference','FontName','arial','FontSize',16)
subplot(2,1,2);
hold on
stem(xcorr(u))
set(gca,'FontName','arial','FontSize',14)
xlabel('lag','FontName','arial','FontSize',16)
ylabel('value','FontName','arial','FontSize',16)

% MOESP法の結果(2出力)
fig2 = figure(2);
set(fig2,'Position',[0 2000 600 700]);
% iptwindowalign(fig1,'right',fig2,'left');
subplot(3,1,1);
hold on
plot(cl_t,cl_int,'LineWidth',2);
plot(cl_t,cl_ini,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',14)
ylabel('voltage [V]','FontName','arial','FontSize',14)
legend('True','CL-MOESP')
set(legend,'FontName','arial','FontSize',12,'Location','southeast')
subplot(3,1,2);
hold on
plot(cl_t,cl_out1t,'LineWidth',2);
plot(cl_t,cl_out1i,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',14)
ylabel('location [m]','FontName','arial','FontSize',14)
legend('True','CL-MOESP')
set(legend,'FontName','arial','FontSize',12,'Location','southeast')
subplot(3,1,3);
hold on
plot(cl_t,cl_out2t,'LineWidth',2);
plot(cl_t,cl_out2i,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',14)
ylabel('angular [deg]','FontName','arial','FontSize',14)
legend('True','CL-MOESP')
set(legend,'FontName','arial','FontSize',12,'Location','southeast')

% saveas(fig1,'figdata/figure1.pdf')
% saveas(fig2,'figdata/figure2.pdf')
