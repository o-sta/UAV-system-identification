%==========================================================================
% ◎ ドローンシミュレーション
% M系列信号の入出力特性を求める
%==========================================================================
close all;
clear;
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

A_val = double(A);
B_val = double(B);
D_val = double(D);

%--------------------------------------------------------------------------
% ● M系列信号生成 v2
%--------------------------------------------------------------------------
% パラメータ
mP = 5; % M系列のmP段シフトレジスタ
mk = 2; % 最終段以外にフィードバックさせる信号 (-mk 番目)


x = randi([0 1], mP, 1);    % 状態の初期値をランダムに決定
x = [1 1 1 1 1]';
u = zeros(2^mP-1, 1);       % 出力変数の定義
for i = 1:2^mP-1            % 信号の生成
    x = [xor(x(mk),x(end));x(1:end-1)]; %フィードバック
    u(i) = x(1);
end
for i = 1:2^mP - 1          % 2値信号の値を0,1から-1,1に変更
    if u(i) == 1
        u(i)=0.00001;
    else
        u(i)=-0.00001;
    end
end

% u_in = timeseries(u, 0:0.4:12, 'name', 'Position');


%u_data = zeros(size(t_data));

% 信号全体の時間[s]
s_t = 5;
% 離散信号1個あたりの継続時間[s]
s_tp = 0.001;
% 1個あたりの連続時間信号数
s_p = round(s_t/s_tp/numel(u));
% 時間信号を作成
t_data(:,1) = 0:s_tp:s_tp*s_p*numel(u)-s_tp;
u_data = zeros(size(t_data));
% 信号を作成
for i = 1:numel(u)
    for j = (i-1)*s_p+1:i*s_p
        u_data(j) = u(i);
    end
end
u_in = timeseries(u_data, t_data, 'name', 'Position');

sim('block_nonlinear',s_t)
sim('block_ss',s_t)

plot_data

% 伝達関数作成 in tx out phi
clear A B C D;
A = A_val;
B = B_val(:,2);
C = [0 0 0 0 0 0 1 0 0 0 0 0];
D = [0];

[num,den] = ss2tf(A,B,C,D);
cP = tf(num,den);

%[y,t] = lsim(cP, u_data, t_data);
%figure(1);
%plot(t,y/(2*pi)*360);


% 以下MOESP法
%--------------------------------------------------------------------------
% ● 伝達関数と出力信号の生成
%--------------------------------------------------------------------------
% パラメータ
m = 1;      %入力の数
p = 1;      %出力の数
sample = 1; %サンプリング周波数
n = 12;      %状態変数の次数
k = 1000;      %データ行列の行数

% フィボナッチ数列
% num = [1 0];              %伝達関数分子
% den = [1 -1 -1];            %伝達関数分母
% dP = tf(num,den,sample);

% 連続系を離散系に双一次変換したもの
% num = [1];                  %伝達関数分子
% den = [1 5];                %伝達関数分母
% cP = tf(num,den);
% dP = c2d(cP,sample,'tustin');
% [y,t] = lsim(dP, u, 0:1:numel(u)-1);
% stem(t,u)siz
% hold on
% stem(t,y)

%--------------------------------------------------------------------------
% ● 入力と出力のデータ行列生成
% k : データ行列の行数
%--------------------------------------------------------------------------
clear A B C D;
%MOESP
% [A, B, C, D] = mf_moesp(Ldata.data(:,14)', Ldata.data(:,7)', k, n);
[A, B, C, D] = mf_moesp(data.data(:,14)', data.data(:,7)', k, n);

%N4SID
% [A, B, C, D] = mf_n4sid(Ldata.data(:,14)', Ldata.data(:,7)', k/2, n);

%--------------------------------------------------------------------------
% ● 伝達関数の算出
%--------------------------------------------------------------------------
[num, den] = ss2tf(A,B,C,D);
dP_est = tf(num,den,0.001);

%データのプロット
dP_est
cP_est = d2c(dP_est,'zoh');
[y,t] = lsim(cP_est, u_data, t_data);
figure(1)
plot(t,y/(2*pi)*360);

%FIT算出
clear num den;
[num, den] = tfdata(cP_est);
sim('block_nonlinear_compare',s_t);

clear y yh ya;
y = Cdata.data(:, 7);
yh = Cdata.data(:, 23);
ya = mean(Cdata.data(:, 7));

fit = (1 - (sqrt(sum((y-yh).^2)))/(sqrt(sum((y-ya).^2))) )*100;




