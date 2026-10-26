close all, clear variables, clc
Gp = tf([0.1006 -0.2299 1.758 0.1326],[1 0.5415 8.667 1.226])
% w = ss(Gp)
v = c2d(Gp, 0.1)
w = ss(v)
U = ctrb(w.A, w.B)
det = det(U)

s1 = -2 + 5.2i;
s2 = -2 - 5.2i;
s3 = -2;
z1 = exp(s1*0.1)
z2 = exp(s2*0.1)
z3 = exp(s3*0.1)
P = [z1 z2 z3];
K = acker(w.A, w.B, P)

syms x1 x2 x3 k0
x = [x1; x2; x3];
u = -K*x;
y = w.C*x;
XX = w.A*x + w.B.*(u + k0*y);
S = solve(XX == 0, [k0 x1 x2 x3]);
k0 = round(S.k0,10)
% 
% matrix = (w.A*x + w.B.*(u))./w.B
% S = solve(matrix == 0, [x1 x2 x3])
