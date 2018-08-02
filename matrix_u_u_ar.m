syms b l d;
syms b11 b12 b13 b14 b21 b22 b23 b24 b31 b32 b33 b34 b41 b42 b43 b44;
syms m Ix Iy Iz;

%変換行列作成
P = diag([b b*l b*l d])
Q = [1 1 1 1;
    1 -1 -1 1;
    1 1 -1 -1;
    -1 1 -1 1]
R = [b11 b12 b13 b14;
    b21 b22 b23 b24;
    b31 b32 b33 b34;
    b41 b42 b43 b44
    ]

X = Q*R
S = P*Q*R

B = [0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    1/m 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz
    ]

B_ar = B*S
