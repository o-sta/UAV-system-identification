%==========================================================================
% ◎ ドローンシミュレーション
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

sim('block_nonlinear')
