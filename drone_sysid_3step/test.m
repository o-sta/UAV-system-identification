close all
% p1 = [0 0 0]; % 始点
% p2 = [1 0 0]; % 終点
% quiver3(p1(1), p1(2), p1(3), p2(1)-p1(1), p2(2)-p1(2), p2(3)-p1(3), 0)
% hold on
% p1 = [0 0 0]; % 始点
% p2 = [0 1 0]; % 終点
% quiver3(p1(1), p1(2), p1(3), p2(1)-p1(1), p2(2)-p1(2), p2(3)-p1(3), 0)
% p1 = [0 0 0]; % 始点
% p2 = [0 0 1]; % 終点
% quiver3(p1(1), p1(2), p1(3), p2(1)-p1(1), p2(2)-p1(2), p2(3)-p1(3), 0)

fig1 = figure(1);
xlabel('x [m]','FontName','arial','FontSize',10)
ylabel('y [m]','FontName','arial','FontSize',10)
zlabel('z [m]','FontName','arial','FontSize',10)
set(gca,...
    'YDir','reverse',...
    'ZDir','reverse',...
    'XLim',[-10 10],...
    'YLim',[-10 10],...
    'YLim',[-10 10],...
    );



rotate3d on;

% ベクトルの更新を行う
function update_vector(vector_state,x,y,z,phi,theta,psi)

end

function vector_state = create_vector(x,y,z,phi,theta,psi)
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
sin_phi = sin(phi);
cos_phi = cos(phi);
sin_theta = sin(theta);
cos_theta = cos(theta);
sin_psi = sin(psi);
cos_psi = cos(psi);
Rx = [1 0 0;0 cos_phi -sin_phi;0 sin_phi cos_phi];
Ry = [cos_theta 0 sin_theta;0 1 0;-sin_theta 0 cos_theta];
Rz = [cos_psi -sin_psi 0;sin_psi cos_psi 0;0 0 1];



p1 = [0 0 0]; % 始点
p2 = [0 0 0]; % 終点
quiver3(p1(1), p1(2), p1(3), p2(1)-p1(1), p2(2)-p1(2), p2(3)-p1(3),0)
vector_state(1:3) = quiver3();

end

% annotation('arrow',[0 0 0],[1 1 1]);







