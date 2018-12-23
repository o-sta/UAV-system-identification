%--------------------------------------------------------------------------
% ●回転行列の式算出
% 慣性系から機体固定系への座標変換を表している。x,y,zの順番に回転させる。
%--------------------------------------------------------------------------
% syms phi theta psi
% cos_phi = cos(phi);
% sin_phi = sin(phi);
% cos_theta = cos(theta);
% sin_theta = sin(theta);
% cos_psi = cos(psi);
% sin_psi = sin(psi);
% Rx = [1 0 0;
%     0 cos_phi -sin_phi;
%     0 sin_phi cos_phi];
% Ry = [cos_theta 0 sin_theta;
%     0 1 0;
%     -sin_theta 0 cos_theta];
% Rz = [cos_psi -sin_psi 0;
%     sin_psi cos_psi 0;
%     0 0 1];
% % 慣性系から機体固定系への回転行列
% e2b_matrix = Rz*Ry*Rx

%--------------------------------------------------------------------------
% ●回転行列と角速度の関係式
% dR=R^T*omegaのR^Tとomegaを入れ替えてたら符号が逆になることを確認している
%--------------------------------------------------------------------------
% syms x y z a b c u v w;
% R = [x a b;
%     a y c;
%     b c z];
% omega = [0 -w v;
%     w 0 -u;
%     -v u 0];
% R*omega
% omega*R

%--------------------------------------------------------------------------
% ●回転行列の微分
% E:慣性系
% B:機体固定系
%--------------------------------------------------------------------------
% syms phi theta psi              % 慣性系から見た機体固定系の角度(E)
% syms dphi dtheta dpsi           % 角度の微分(E)
% syms omega_x omega_y omega_z    % 角速度(B)
% cos_phi = cos(phi);
% sin_phi = sin(phi);
% cos_theta = cos(theta);
% sin_theta = sin(theta);
% cos_psi = cos(psi);
% sin_psi = sin(psi);
% Rx = [1 0 0;
%     0 cos_phi sin_phi;
%     0 -sin_phi cos_phi];
% Ry = [cos_theta 0 -sin_theta;
%     0 1 0;
%     sin_theta 0 cos_theta];
% Rz = [cos_psi sin_psi 0;
%     -sin_psi cos_psi 0;
%     0 0 1];
% % 慣性系から機体固定系への回転行列
% b2e_matrix = Rx*Ry*Rz;
% 
% % 論文
% % Rx = [1 0 0;
% %     0 cos_phi -sin_phi;
% %     0 sin_phi cos_phi];
% % Ry = [cos_theta 0 sin_theta;
% %     0 1 0;
% %     -sin_theta 0 cos_theta];
% % Rz = [cos_psi -sin_psi 0;
% %     sin_psi cos_psi 0;
% %     0 0 1];
% % % 慣性系から機体固定系への回転行列
% % b2e_matrix = Rz*Ry*Rx;
% 
% diff_b2e_matrix ...
%     = diff(b2e_matrix,phi)*dphi ...
%     + diff(b2e_matrix,theta)*dtheta ...
%     + diff(b2e_matrix,psi)*dpsi;
% 
% omega_b_cross = transpose(b2e_matrix)*diff_b2e_matrix;
% omega_b_cross = simplify(omega_b_cross);
% eqn1 = omega_x == -omega_b_cross(2,3);
% eqn2 = omega_y ==  omega_b_cross(1,3);
% eqn3 = omega_z == -omega_b_cross(1,2);
% 
% [e2b_T_matrix,B] = equationsToMatrix([eqn1, eqn2, eqn3], [dphi, dtheta, dpsi]);
% b2e_T_matrix = e2b_T_matrix^(-1);
% simplify(b2e_T_matrix)

%--------------------------------------------------------------------------
% ● 運動方程式を成分に展開する
%--------------------------------------------------------------------------
syms phi theta psi              % 慣性系から見た機体固定系の角度(E)
syms dphi dtheta dpsi           % 角度の微分(E)
syms p q r                      % 角速度(B)
syms u v w                      % 速度(B)
syms du dv dw                   % 速度の微分(B)
syms m g f_t                    % 質量 重力加速度 推力
syms f_wx f_wy f_wz             % 風による外乱(B)

% 回転行列の定義
cos_phi = cos(phi);
sin_phi = sin(phi);
cos_theta = cos(theta);
sin_theta = sin(theta);
cos_psi = cos(psi);
sin_psi = sin(psi);
Rx = [1 0 0;
    0 cos_phi sin_phi;
    0 -sin_phi cos_phi];
Ry = [cos_theta 0 -sin_theta;
    0 1 0;
    sin_theta 0 cos_theta];
Rz = [cos_psi sin_psi 0;
    -sin_psi cos_psi 0;
    0 0 1];
R = Rx*Ry*Rz;

%論文
% Rx = [1 0 0;
%     0 cos_phi -sin_phi;
%     0 sin_phi cos_phi];
% Ry = [cos_theta 0 sin_theta;
%     0 1 0;
%     -sin_theta 0 cos_theta];
% Rz = [cos_psi -sin_psi 0;
%     sin_psi cos_psi 0;
%     0 0 1];
% % 慣性系から機体固定系への回転行列
% R = Rz*Ry*Rx;

T = ... % 角速度の変換行列
[              cos(psi)/cos(theta),              sin(psi)/cos(theta), 0;
                        -sin(psi),                         cos(psi), 0;
 (cos(psi)*sin(theta))/cos(theta), (sin(psi)*sin(theta))/cos(theta), 1];

% ベクトルの定義
e_z = transpose([0 0 1]);
b_z = transpose([0 0 1]);
omega_B = transpose([p q r]);
v_B = transpose([u v w]);
dv_B = transpose([du dv dw]);
f_wB = transpose([f_wx f_wy f_wz]);
f_B = m*g*transpose(R)*e_z - f_t*b_z + f_wB;


eq1 = m*(cross(omega_B,v_B) + dv_B);