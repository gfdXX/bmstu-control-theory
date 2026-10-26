clc;
close all;
num = [0 0.1769 0.2875];
den = [1 0.4149 0.4693];
T = 1;
sys_d = tf(num, den, T);
z = tf('z');
% zpk(sys_d);
sys_c = d2c(sys_d, 'zoh');
sys_d2 = c2d(sys_c, T, 'zoh');
zpk(sys_c);
figure, step(sys_d, sys_c, sys_d2);
figure, step(sys_d2);
legend('Разомкнутая система')
figure, step(sys_c);
legend('Разомкнутая непрерывная система')
% 
% 
pidtune(sys_c, 'PID')


% continuous pid
Kp = 0.615;
Ti = 0.523;
Td = 0.008;
% Kp = 1.4;
% Ti = 0.9;
% Td = 0.5;
s = tf('s');
figure, step(sys_c);
Gd = Kp*(1 + 1/(Ti*s) + Td*s);
cl = feedback(Gd*sys_c, 1);
figure, step(cl);
legend('Замкнутая непрерывная система')
stepinfo(cl)

%discrete pid
K = Kp - Kp/(2*Ti);
Ki = Kp/Ti;
Kd = Kp * Td;
z = tf('z');
Gd = K + Ki/(1-z^(-1)) + Kd*(1 - z^(-1));
cd = feedback(sys_d2*Gd, 1);
figure, step(cd)
legend('Замкнутая дискретная система')
stepinfo(cd)
G = feedback(Gd, sys_d2);
figure, step(G)
legend('Управляющее воздействие')
stepinfo(G)

% aperiodic controller
Gc = (2.153 + 0.8932*z^(-1) + 1.011*z^(-2))/(1 - 0.3808*z^(-1) - 0.6189*z^(-2));
cap = feedback(sys_d2*Gc, 1);
figure, step(cap)
legend('Замкнутая система')
stepinfo(cap)
G = feedback(Gc, sys_d2);
figure, step(G)
legend('Управляющее воздействие')
stepinfo(G)

%controller & full estimator
% G = [0 1; -0.4693 -0.4149];
% H = [0; 1];
% C = [0.2875 0.1769];
% % GH = G*H;
% % CG = C*G;
% % [H GH]
% % [C; CG]
% % rank([H GH]) % =2 ok
% % rank([C; CG]) % =2 ok
% % pc = [complex(0.4, 0.7), complex(0.4, -0.7), -0.7];
% pc = [complex(0.2, 0.5), complex(0.2, -0.5)];
% K = acker(G, H, pc)
% % (z*eye(3)-(G-H*K))^(-1)
% sreg = ss(G - H*K, H, C, [], T);
% % f = C*((z*eye(3) - (G - H*K))^(-1))*H;
% % Num = f.Numerator;
% % Den = f.Denominator;
% % syms z
% % fsym = poly2sym(cell2mat(Num),z)/poly2sym(cell2mat(Den),z);
% % K0 = 1/double(subs(fsym, z, 1));
% % figure, step(sreg*K0)
% % stepinfo(sreg*K0)
% pe = [-0.1, 0.1];
% Ge = [0 0.4693; 1 -0.4149;];
% Ce = [0 1];
% He = [0.2875; 0.1769];
% rank([Ce; Ce*Ge]); % =2 ok
% phi = (Ge - pe(1))*(Ge - pe(2));
% Ke = phi*([Ce; Ce*Ge])^(-1)*[0; 1];
% % sysest = ss(Ge - Ke*Ce, He, Ce, [], 1);
% % figure, step(sysest)
% % stepinfo(sysest)
% % sysest = ss(G - Ke*C, H, C, [], 1);
% % figure, step(sysest)
% Gtotal = G - Ke*C - H*K;
% eigs(Gtotal)
% systotal = ss(Gtotal, H, C, [], 1);
% figure, step(systotal)
% legend('Замкнутая система')
% stepinfo(systotal)
% z = tf('z');
% f = C*((z*eye(2) - Gtotal)^(-1))*H;
% Num = f.Numerator;
% Den = f.Denominator;
% syms z
% fsym = poly2sym(cell2mat(Num),z)/poly2sym(cell2mat(Den),z);
% K0 = 1/double(subs(fsym, z, 1));
% figure, step(systotal*K0)
% legend('Замкнутая система')
% stepinfo(systotal*K0)
% z = tf('z');
% u = -K*((z*eye(2) - Gtotal)^(-1))*Ke;
% figure, step(u)
% legend('Управляющее воздействие')
% 
% %controller & min estimator
% G = [0 1; -0.4693 -0.4149];
% H = [0; 1];
% C = [0.2875 0.1769];
% GH = G*H;
% [H GH];
% rank([H GH]); % =2 ok
% %pc = [complex(0.4, 0.7), complex(0.4, -0.7), -0.6];
% pc = [complex(0.2, 0.5), complex(0.2, -0.5)];
% K = acker(G, H, pc);
% (z*eye(2)-(G-H*K))^(-1)
% sreg = ss(G - H*K, H, C, [], T);
% f = C*((z*eye(2) - (G - H*K))^(-1))*H;
% Num = f.Numerator;
% Den = f.Denominator;
% syms z
% fsym = poly2sym(cell2mat(Num),z)/poly2sym(cell2mat(Den),z);
% K0 = 1/double(subs(fsym, z, 1));
% figure, step(sreg*K0)
% stepinfo(sreg*K0)
% Ge = G';
% Ce = [0 1];
% He = [0.2875 0.1769]';
% Gaa = 0;
% Gab = 0.4693;
% Gba = 1;
% Gbb = 0.4149;
% Ha = 0.1769;
% Hb = 0.2875;
% Cm = [1 0];
% CG = Ce*Ge;
% % C2G = Ce*(Ge*Ge);
% GH = Ge*He;
% % G2H = Ge*Ge*He;
% rank([He GH]); % = 3 ok
% rank([Ce; CG]); % =3 ok
% pe2 = 0.6;
% phi = (Gbb - pe2(1));
% Ke = phi*Gab*Gbb^(-1);
% corrK = Ke*Gab;
% corrG = corrK;
% Gest = G - corrG;
% 
% sysest = ss(Gest, He, Ce, [], 1);
% figure, step(sysest)
% corrG2 = corrK;
% Gtotal = G - corrG2 - H*K;
% eigs(Gtotal)
% % stepinfo(sysest)
% systotal = ss(Gtotal, H, C, [], 1);
% figure, step(systotal)
% legend('Замкнутая система')
% stepinfo(systotal)
% z = tf('z');
% f = C*((z*eye(2) - Gtotal)^(-1))*H;
% Num = f.Numerator;
% Den = f.Denominator;
% syms z
% fsym = poly2sym(cell2mat(Num),z)/poly2sym(cell2mat(Den),z);
% K0 = 1/double(subs(fsym, z, 1))
% figure, step(systotal*K0)
% legend('Замкнутая система')
% stepinfo(systotal*K0)
% z = tf('z');
% u = -K*((z*eye(2) - Gtotal)^(-1))*[0; Ke];
% figure, step(u)
% legend('Управляющее воздействие')