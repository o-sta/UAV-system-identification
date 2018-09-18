%==========================================================================
% ◎ ドローンシミュレーション
% M系列信号の入出力特性を求める
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
l = 0.1785
q = 2.423*19^(-7);

A_val = double(A);
B_val = double(B);
D_val = double(D);

%--------------------------------------------------------------------------
% ● 伝達関数と出力信号の生成
%--------------------------------------------------------------------------
% パラメータ
m = 1;          % 入力の数
p = 1;          % 出力の数
sample = 1;     % サンプリング周波数
n = 2;          % 状態変数の次数
k = 100;        % データ行列の行数
ts = 0.001;      % サンプリング時間

%--------------------------------------------------------------------------
% ● シミュレーション実行
%--------------------------------------------------------------------------
rpm = 551.3;
sim('block/drone_nonlinear_full')

rpm/(2*pi)*60

plot(data.time,data.data(:,3))



% 
% %--------------------------------------------------------------------------
% % ● M系列信号生成 v2
% %--------------------------------------------------------------------------
% mP = 5;     % M系列のmP段シフトレジスタ
% mk = 2;     % 最終段以外にフィードバックさせる信号
% 
% x = randi([0 1], mP, 1);    % 状態の初期値をランダムに決定
% x = [0 1 0 0 1]';           % 状態の初期値を指定（全て0は禁止）
% u = zeros(2^mP-1, 1);       % 出力変数の定義
% for i = 1:2^mP-1            % 信号の生成
%     x = [xor(x(mk),x(end));x(1:end-1)]; % フィードバック
%     u(i) = x(1);
% end
% for i = 1:2^mP - 1          % 2値信号の値を0,1から他の値に変更
%     if u(i) == 1
%         u(i)=0.00001;
%     else
%         u(i)=-0.00001;
%     end
% end
% s_t = 5;        % 信号全体の時間[s]
% s_tp = 0.001;   % 離散信号1個あたりの継続時間[s] / サンプリング時間
% s_p = round(s_t/s_tp/numel(u));               % 1個あたりの連続時間信号数
% t_data(:,1) = 0:s_tp:s_tp*s_p*numel(u)-s_tp;  % 時間信号を作成
% u_data = zeros(size(t_data));
% for i = 1:numel(u)  % 信号を作成
%     for j = (i-1)*s_p+1:i*s_p
%         u_data(j) = u(i);
%     end
% end
% u_in = timeseries(u_data, t_data, 'name', 'Position');  % 時系列作成
% sim('block/drone_nonlinear',s_t)
% sim('block/block_ss',s_t)
% 
% %--------------------------------------------------------------------------
% % ● MOESP,N4SID実行
% %--------------------------------------------------------------------------
% clear A B C D;
% % 離散データ
% [moA, moB, moC, moD] = mf_moesp(data_D.data(:,14)', data_D.data(:,7)', k, n);   %MOESP
% [n4A, n4B, n4C, n4D] = mf_n4sid(data_D.data(:,14)', data_D.data(:,7)', k/2, n); %N4SID
% %--------------------------------------------------------------------------
% % ● 伝達関数の算出 moesp
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
% sim('block/drone_nonlinear_compare',s_t);
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
% sim('block/drone_nonlinear_compare',s_t);
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
