%==========================================================================
% ◎ ドローンの線形近似
%==========================================================================
clear;
close all;

syms x y z u v w ph th ps p q r;  % 状態変数
syms dx dy dz du dv dw dph dth dps dp dq dr; % 状態変数の微分
syms fwx fwy fwz                  % 風による力
syms twx twy twz                  % 風によるトルク
syms ft tx ty tz                  % 入力推力とトルク
syms Ix Iy Iz m g                 % 慣性モーメントと質量、重力加速度

%--------------------------------------------------------------------------
% ● 非線形状態方程式
%--------------------------------------------------------------------------
eq = [
dx == w*(sin(ph)*sin(ps) + cos(ph)*cos(ps)*sin(th)) - v*(cos(ph)*sin(ps) - cos(ps)*sin(ph)*sin(th)) + u*(cos(ps)*cos(th));
dy == v*(cos(ph)*cos(ps) + sin(ph)*sin(ps)*sin(th)) - w*(cos(ps)*sin(ph) - cos(ph)*sin(ps)*sin(th)) + u*(cos(th)*sin(ps));
dz == w*(cos(ph)*cos(th)) - u*(sin(th)) + v*(cos(th)*sin(ph));
du == r*v - q*w + g*(sin(th)) + fwx / m;
dv == p*w - r*u - g*(sin(ph)*cos(th)) + fwy / m;
dw == q*u - p*v - g*(cos(th)*cos(ph)) + (fwz + ft) / m;
dph == p + r*(cos(ph)*tan(th)) + q*(sin(ph)*tan(th));
dth == q*(cos(th)) - r*(sin(ph));
dps == r*(cos(ph)/cos(th)) + q*(sin(ph)/cos(th));
dp == r*q*(Iy - Iz)/Ix + (tx + twx)/Ix;
dq == p*r*(Iz - Ix)/Iy + (ty + twy)/Iy;
dr == p*q*(Ix - Iy)/Iz + (tz + twz)/Iz
];
% eq(1,1) = dx == w*(sin(ph)*sin(ps) + cos(ph)*cos(ps)*sin(th)) - v*(cos(ph)*sin(ps) - cos(ps)*sin(ph)*sin(th)) + u*(cos(ps)*cos(th));
% eq(2,1) = dy == v*(cos(ph)*cos(ps) + sin(ph)*sin(ps)*sin(th)) - w*(cos(ps)*sin(ph) - cos(ph)*sin(ps)*sin(th)) + u*(cos(th)*sin(ps));
% eq(3,1) = dz == w*(cos(ph)*cos(th)) - u*(sin(th)) + v*(cos(th)*sin(ph));
% eq(4,1) = du == r*v - q*w + g*(sin(th)) + fwx / m;
% eq(5,1) = dv == p*w - r*u - g*(sin(ph)*cos(th)) + fwy / m;
% eq(6,1) = dw == q*u - p*v - g*(cos(th)*cos(ph)) + (fwy + ft) / m;
% eq(7,1) = dph == p + r*(cos(ph)*tan(th)) + q*(sin(ph)*tan(th));
% eq(8,1) = dth == q*(cos(th)) - r*(sin(ph));
% eq(9,1) = dps == r*(cos(ph)/cos(th)) + q*(sin(ph)/cos(th));
% eq(10,1) = dp == r*q*(Iy - Iz)/Ix + (tx + twx)/Ix;
% eq(11,1) = dq == p*r*(Iz - Ix)/Iy + (ty + twy)/Iy;
% eq(12,1) = dr == p*q*(Ix - Iy)/Iz + (tz + twz)/Iz;

%--------------------------------------------------------------------------
% ● 三角関数の線形近似
%--------------------------------------------------------------------------
eq = subs(eq, cos(ph), 1);
eq = subs(eq, cos(th), 1);
eq = subs(eq, cos(ps), 1);
eq = subs(eq, sin(ph), ph);
eq = subs(eq, sin(th), th);
eq = subs(eq, sin(ps), ps);
eq = subs(eq, tan(ph), ph);
eq = subs(eq, tan(th), th);
eq = subs(eq, tan(ps), ps);
% for i = 1:numel(eq)
%   eq(i) = subs(eq(i), cos(ph), 1);
%   eq(i) = subs(eq(i), cos(th), 1);
%   eq(i) = subs(eq(i), cos(ps), 1);
%   eq(i) = subs(eq(i), sin(ph), ph);
%   eq(i) = subs(eq(i), sin(th), th);
%   eq(i) = subs(eq(i), sin(ps), ps);
%   eq(i) = subs(eq(i), tan(ph), ph);
%   eq(i) = subs(eq(i), tan(th), th);
%   eq(i) = subs(eq(i), tan(ps), ps);
% end

%--------------------------------------------------------------------------
% ● テイラー展開による線形化及び行列 A の算出
%--------------------------------------------------------------------------
% rhs(eq(1).e)
ssx = [x y z u v w ph th ps p q r]; % 状態変数
ssx_Equilibrium_point = [x y z 0 0 0 0 0 0 0 0 0]; % 平衡点
A = zeros(numel(eq),numel(ssx));
A = sym(A);
for i = 1:numel(eq)
  for j = 1:numel(ssx)
    A(i,j) = diff(rhs(eq(i)), ssx(j));
  end
end
% 平衡点代入
for i = 1:numel(ssx)
  A = subs(A, ssx(i), ssx_Equilibrium_point(i));
end

%--------------------------------------------------------------------------
% ● テイラー展開による線形化及び行列 B の算出
%--------------------------------------------------------------------------
ssu = [ft tx ty tz];
B = zeros(numel(eq),numel(ssu));
B = sym(B);
for i = 1:numel(eq)
  for j = 1:numel(ssu)
    B(i,j) = diff(rhs(eq(i)), ssu(j));
  end
end
% 平衡点代入
for i = 1:numel(ssx)
  B = subs(B, ssx(i), ssx_Equilibrium_point(i));
end

%--------------------------------------------------------------------------
% ● テイラー展開による線形化及び行列 D の算出
%--------------------------------------------------------------------------
ssd = [fwx fwy fwz twx twy twz];
D = zeros(numel(eq),numel(ssd));
D = sym(B);
for i = 1:numel(eq)
  for j = 1:numel(ssd)
    D(i,j) = diff(rhs(eq(i)), ssd(j));
  end
end
% 平衡点代入
for i = 1:numel(ssx)
  D = subs(D, ssx(i), ssx_Equilibrium_point(i));
end

%--------------------------------------------------------------------------
% ● データ保存
%--------------------------------------------------------------------------

save('data/drone_ss','A','B','D');









%
