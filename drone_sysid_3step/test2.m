% 回転行列の式算出

syms phi theta psi

cos_phi = cos(phi);
sin_phi = sin(phi);

cos_theta = cos(theta);
sin_theta = sin(theta);

cos_psi = cos(psi);
sin_psi = sin(psi);

Rx = [1 0 0;0 cos_phi -sin_phi;0 sin_phi cos_phi];
Ry = [cos_theta 0 sin_theta;0 1 0;-sin_theta 0 cos_theta];
Rz = [cos_psi -sin_psi 0;sin_psi cos_psi 0;0 0 1];

% 慣性系から機体固定系への回転行列
e2b_matrix = Rz*Ry*Rx

