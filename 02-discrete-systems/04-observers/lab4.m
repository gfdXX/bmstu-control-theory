close all, clear variables, clc
A = [-34 -54 -17; 20 30.5 9.4; -5.9 -7.9 -2.3];
B = [0.228; -0.127; 0.038];
C = [21000 63000 84000];
D = 0;
w = ss(A, B, C, D);
T0 = 10;
w = c2d(w, T0)
N = obsv(w.A, w.C)
det = det(N)

s1 = -1.8+4.68*i;
s2 = -1.8-4.68*i;
s3 = -6;
z1 = exp(s1*T0)
z2 = exp(s2*T0)
z3 = exp(s3*T0)
P = [z1 z2 z3];
K = acker(w.A, w.B, P)

s1n = -5; 
s2n = -5; 
s3n = -5;
z1n = exp(s1n*T0)
z2n = exp(s2n*T0)
z3n = exp(s3n*T0)
P2 = [z1n z2n z3n];
L = acker(w.A', w.C', P2);
L = L'

syms x1 x2 x3 k0
x = [x1; x2; x3];
u = -K*x;
y = w.C*x;
XX = w.A*x + w.B.*(u + k0*y);
S = solve(XX == 0, [k0 x1 x2 x3]);
k0 = round(S.k0,5)
