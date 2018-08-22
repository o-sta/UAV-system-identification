function [dx, dy, dz, du, dv, dw, dph, dth, dps, dp, dq, dr]  = Drone_nonlinear_ss(u, v, w, ph, th, ps, p, q, r, ft, tx, ty, tz, fwx, fwy, fwz, twx, twy, twz)
%DRONE_NONLINEAR_SS この関数の概要をここに記述
%   詳細説明をここに記述
dx = w*(sin(ph)*sin(ps) + cos(ph)*cos(ps)*sin(th)) - v*(cos(ph)*sin(ps) - cos(ps)*sin(ph)*sin(th)) + u*(cos(ps)*cos(th));
dy = v*(cos(ph)*cos(ps) + sin(ph)*sin(ps)*sin(th)) - w*(cos(ps)*sin(ph) - cos(ph)*sin(ps)*sin(th)) + u*(cos(th)*sin(ps));
dz = w*(cos(ph)*cos(th)) - u*(sin(th)) + v*(cos(th)*sin(ph));
du = r*v - q*w + g*(sin(th)) + fwx / m;
dv = p*w - r*u - g*(sin(ph)*cos(th)) + fwy / m;
dw = q*u - p*v - g*(cos(th)*cos(ph)) + (fwz + ft) / m;
dph = p + r*(cos(ph)*tan(th)) + q*(sin(ph)*tan(th));
dth = q*(cos(th)) - r*(sin(ph));
dps = r*(cos(ph)/cos(th)) + q*(sin(ph)/cos(th));
dp = r*q*(Iy - Iz)/Ix + (tx + twx)/Ix;
dq = p*r*(Iz - Ix)/Iy + (ty + twy)/Iy;
dr = p*q*(Ix - Iy)/Iz + (tz + twz)/Iz;
end

