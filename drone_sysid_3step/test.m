close all
clear

frame_figure = figure(1);
hold on;
xlabel('x [m]','FontName','arial','FontSize',10)
ylabel('y [m]','FontName','arial','FontSize',10)
zlabel('z [m]','FontName','arial','FontSize',10)
set(gca,...
    'YDir','reverse',...
    'ZDir','reverse',...
    'XLim',[-4 4],...
    'YLim',[-4 4],...
    'ZLim',[-4 4]...
    );
frame_axes = gca;
grid on;
earth_frame = create_frame(frame_axes,0,0,0,0*pi/180,0*pi/180,30*pi/180);
body_frame = create_frame(frame_axes,0,0,0,30*pi/180,0*pi/180,30*pi/180);
rotate3d on;

% 機体固定座標系の更新
% ローカル座標系(地図座標系)から見た機体固定系の座標と姿勢を更新する
function update_frame(frame_state,x,y,z,phi,theta,psi)
    % ローカル座標系の定義
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    % 三角関数の定義
    sin_phi = sin(phi);
    cos_phi = cos(phi);
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    sin_psi = sin(psi);
    cos_psi = cos(psi);
    % 回転行列
    Rx = [1 0 0;0 cos_phi -sin_phi;0 sin_phi cos_phi];
    Ry = [cos_theta 0 sin_theta;0 1 0;-sin_theta 0 cos_theta];
    Rz = [cos_psi -sin_psi 0;sin_psi cos_psi 0;0 0 1];
    % 機体固定系の算出
    bx = Rx*Ry*Rz*ex;
    by = Rx*Ry*Rz*ey;
    bz = Rx*Ry*Rz*ez;
    % 座標系ベクトルの設定
    set(frame_state(1),'UData',bx(1),'VData',bx(2),'WData',bx(3),'XData',x,'YData',y,'ZData',z);
    set(frame_state(2),'UData',by(1),'VData',by(2),'WData',by(3),'XData',x,'YData',y,'ZData',z);
    set(frame_state(3),'UData',bz(1),'VData',bz(2),'WData',bz(3),'XData',x,'YData',y,'ZData',z);
end

% 座標系の作成
% 全ての引数を0にした場合、ローカル座標系となる
function frame_state = create_frame(frame_axes,x,y,z,phi,theta,psi)
    % 座標系のプロット
    frame_state(1) = quiver3(frame_axes,0,0,0,0,0,0,0,'Color','red');
    frame_state(2) = quiver3(frame_axes,0,0,0,0,0,0,0,'Color','blue');
    frame_state(3) = quiver3(frame_axes,0,0,0,0,0,0,0,'Color','green');
    update_frame(frame_state,x,y,z,phi,theta,psi);
end
