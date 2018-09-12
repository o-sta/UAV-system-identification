%==========================================================================
% ◎ システム同定シミュレーション（倒立振子,下向き基準）
%==========================================================================
close all;
clear;
addpath('SI_Function');
cdip_para;
%--------------------------------------------------------------------------
% ● 状態空間表現の定義(振子上向き基準)
%--------------------------------------------------------------------------
% tmp = Jp+mp*lp^2;
% A = [0 0 1 0;
%     0 0 0 1;
%     0 0 -ac 0;
%     0 mp*g*lp/tmp mp*lp*ac/tmp -mup/tmp];
% B = [0;
%     0;
%     bc;
%     -mp*lp*bc/tmp];
% C = [1 0 0 0;
%     0 1 0 0];
% D = [0;
%     0];
% clear tmp;

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

% %--------------------------------------------------------------------------
% % ● 状態空間表現の定義(振子下向き基準)
% %--------------------------------------------------------------------------
% tmp = Jp+mp*lp^2;
% A = [0 1;
%     -6 5];
% B = [0;
%      1];
% C = [1 0;
%      0 1];
% D = [0;
%     0];
% clear tmp;

%--------------------------------------------------------------------------
% ● 伝達関数と出力信号の生成
%--------------------------------------------------------------------------
% パラメータ
m = 1;          % 入力の数
p = 2;          % 出力の数
sample = 1;     % サンプリング周波数
n = 4;          % 状態変数の次数
k = 50;        % データ行列の行数
ts = 0.02;      % サンプリング時間



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

% 連続データ
% [moA, moB, moC, moD] = mf_moesp(data.data(:,14)', data.data(:,7)', k, n);   %MOESP
% [n4A, n4B, n4C, n4D] = mf_n4sid(data.data(:,14)', data.data(:,7)', k/2, n); %N4SID
% 離散データ
[moA, moB, moC, moD] = mf_moesp(input_data, output_data, k, n);   %MOESP
[n4A, n4B, n4C, n4D] = mf_n4sid(input_data, output_data, k/2, n); %N4SID

%--------------------------------------------------------------------------
% ● 同定結果の比較
%--------------------------------------------------------------------------
dP_est = ss(moA, moB, moC, moD, ts);
% dP_est = ss(n4A, n4B, n4C, n4D, ts);
cP_est = d2c(dP_est, 'zoh');
[coA, coB, coC, coD] = ssdata(cP_est);
sim('block_pen_linear_compare',s_t);

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
