%==========================================================================
% ◎ 閉ループシステム同定シミュレーション（倒立振子,上向き基準）
%==========================================================================
close all;
clear;
addpath('SI_Function');
cdip_para;

%--------------------------------------------------------------------------
% ● 状態空間表現の定義(振子下向き基準)
%--------------------------------------------------------------------------
tmp = Jp+mp*lp^2;
A = [0 0 1 0;
    0 0 0 1;
    0 0 -ac 0;
    0 -mp*g*lp/tmp -mp*lp*ac/tmp -mup/tmp];
B = [0;
    0;
    bc;
    mp*lp*bc/tmp];
C = [1 0 0 0;
    0 1 0 0];
D = [0;
    0];
clear tmp;

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
mP = 5;     % M系列のmP段シフトレジスタ
mk = 2;     % 最終段以外にフィードバックさせる信号

x = randi([0 1], mP, 1);    % 状態の初期値をランダムに決定
x = [0 1 0 0 1]';           % 状態の初期値を指定（全て0は禁止）
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
sim('block_pen_linear',s_t);

%--------------------------------------------------------------------------
% ● MOESP,N4SID実行
%--------------------------------------------------------------------------
input_data = data_D.data(:, 3)';
output_data = data_D.data(:,1:2)';
[moA, moB, moC, moD] = mf_moesp(input_data, output_data, k, n);   %MOESP
[n4A, n4B, n4C, n4D] = mf_n4sid(input_data, output_data, k/2, n); %N4SID

%--------------------------------------------------------------------------
% ● 同定結果の比較 MOESP
%--------------------------------------------------------------------------
dP_est_moesp = ss(moA, moB, moC, moD, ts);
cP_est_moesp = d2c(dP_est_moesp, 'zoh');
[coA, coB, coC, coD] = ssdata(cP_est_moesp);
sim('block_pen_linear_compare',s_t);

mo_t = Cdata.time;                         % 時間
mo_in = Cdata.data(:,1);                   % 入力
mo_out1t = Cdata.data(:,2);                % 出力1 真値
mo_out2t = Cdata.data(:,3)/(2*pi)*360;     % 出力2 真値
mo_out1i = Cdata.data(:,4);                % 出力1 同定
mo_out2i = Cdata.data(:,5)/(2*pi)*360;     % 出力2 同定

mofit1 = mf_fit(mo_out1i,mo_out1t)
mofit2 = mf_fit(mo_out2i,mo_out2t)

%--------------------------------------------------------------------------
% ● 同定結果の比較 N4SID
%--------------------------------------------------------------------------
dP_est_n4sid = ss(n4A, n4B, n4C, n4D, ts);
cP_est_n4sid = d2c(dP_est_n4sid, 'zoh');
[coA, coB, coC, coD] = ssdata(cP_est_n4sid);
sim('block_pen_linear_compare',s_t);

n4_t = Cdata.time;                         % 時間
n4_in = Cdata.data(:,1);                   % 入力
n4_out1t = Cdata.data(:,2);                % 出力1 真値
n4_out2t = Cdata.data(:,3)/(2*pi)*360;     % 出力2 真値
n4_out1i = Cdata.data(:,4);                % 出力1 同定
n4_out2i = Cdata.data(:,5)/(2*pi)*360;     % 出力2 同定

n4fit1 = mf_fit(n4_out1i,n4_out1t)
n4fit2 = mf_fit(n4_out2i,n4_out2t)

%--------------------------------------------------------------------------
% ● データのプロット
%--------------------------------------------------------------------------
% 入力信号と自己相関関数
fig1 = figure(1);
movegui('northwest')
subplot(2,1,1);
hold on
plot(mo_t,mo_in,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('voltage [V]','FontName','arial','FontSize',16)
subplot(2,1,2);
hold on
stem(xcorr(u))
set(gca,'FontName','arial','FontSize',14)
xlabel('lag','FontName','arial','FontSize',16)
ylabel('value','FontName','arial','FontSize',16)

% MOESP法の結果(2出力)
fig2 = figure(2);
iptwindowalign(fig1,'right',fig2,'left');
subplot(2,1,1);
hold on
plot(mo_t,mo_out1t,'LineWidth',2);
plot(mo_t,mo_out1i,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('location [m]','FontName','arial','FontSize',16)
legend('True','MOESP')
set(legend,'FontName','arial','FontSize',16,'Location','southeast')
subplot(2,1,2);
hold on
plot(mo_t,mo_out2t,'LineWidth',2);
plot(mo_t,mo_out2i,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('angular [deg]','FontName','arial','FontSize',16)
legend('True','MOESP')
set(legend,'FontName','arial','FontSize',16,'Location','southeast')

% N4SID法の結果(2出力)
fig3 = figure(3);
iptwindowalign(fig2,'right',fig3,'left');
subplot(2,1,1);
hold on
plot(n4_t,n4_out1t,'LineWidth',2);
plot(n4_t,n4_out1i,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('location [m]','FontName','arial','FontSize',16)
legend('True','N4SID')
set(legend,'FontName','arial','FontSize',16,'Location','southeast')
subplot(2,1,2);
hold on
plot(n4_t,n4_out2t,'LineWidth',2);
plot(n4_t,n4_out2i,'--','LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('time [s]','FontName','arial','FontSize',16)
ylabel('angular [deg]','FontName','arial','FontSize',16)
legend('True','N4SID')
set(legend,'FontName','arial','FontSize',16,'Location','southeast')

saveas(fig1,'figdata/figure1.pdf')
saveas(fig2,'figdata/figure2.pdf')
saveas(fig3,'figdata/figure3.pdf')

% %--------------------------------------------------------------------------
% % ● 状態空間表現の算出 moesp
% %--------------------------------------------------------------------------
% [num, den] = ss2tf(moA,moB,moC,moD);
% dP_est = tf(num,den,ts);
%
% %データのプロット
% dP_est
% cP_est = d2c(dP_est,'zoh');
% [mo_y,mo_t] = lsim(cP_est, u_data, t_data);
%
% %FIT算出
% clear num den;
% [num, den] = tfdata(cP_est);
% sim('block_nonlinear_compare',s_t);
%
% clear y yh ya;
% y = Cdata.data(:, 7);
% yh = Cdata.data(:, 23);
% ya = mean(Cdata.data(:, 7));
%
% mofit = (1 - (sqrt(sum((y-yh).^2)))/(sqrt(sum((y-ya).^2))) )*100;
%
% pdt_mt = Cdata.time;
% pdt_m = Cdata.data(:, 23);
% pdt_true = Cdata.data(:, 7);
%
% %--------------------------------------------------------------------------
% % ● 伝達関数の算出 n4sid
% %--------------------------------------------------------------------------
% clear num den;
% [num, den] = ss2tf(n4A,n4B,n4C,n4D);
% dP_est = tf(num,den,ts);
%
% %データのプロット
% dP_est
% cP_est = d2c(dP_est,'zoh');
% [n4_y,n4_t] = lsim(cP_est, u_data, t_data);
%
%
% %FIT算出
% clear num den;
% [num, den] = tfdata(cP_est);
% sim('block_nonlinear_compare',s_t);
%
% clear y yh ya;
% y = Cdata.data(:, 7);
% yh = Cdata.data(:, 23);
% ya = mean(Cdata.data(:, 7));
%
% n4fit = (1 - (sqrt(sum ((y-yh).^2)))/(sqrt(sum((y-ya).^2))) )*100;
%
% pdt_nt = Cdata.time;
% pdt_n = Cdata.data(:, 23);
%
% mofit
% n4fit
%
% plot_data
%
