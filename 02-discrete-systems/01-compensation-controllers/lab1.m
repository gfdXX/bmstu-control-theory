close all, clear variables, clc
Gp = tf([0.1006 -0.2299 1.758 0.1326],[1 0.5415 8.667 1.226])
Gpd = c2d(Gp,2)
Gst = 1/Gpd % регулятор для разомкнутой системы

figure(1)
step(Gp)
hold on
step(Gpd)
grid on
legend("Непрерывный объект", "Дискретный объект");

figure(2)
Gout = Gpd*Gst;
step(Gout)
grid on
% ylim([0 2])
% xlim([0 2.5])

z = tf('z');
Gw = 1/z;
GR = 1/Gpd * Gw / (1-Gw) % регулятор для замкнутой системы
Gww = GR*Gpd/(1+GR*Gpd)
Gop = Gst*Gpd

figure(3)
Gn=1/(1+Gpd*GR);
step(Gn)
grid on

% linearSystemAnalyzer(Gn)
ltiview({'step', 'pzmap', 'bode', 'nyquist'}, Gout)
ltiview({'step', 'pzmap', 'bode', 'nyquist'}, Gww)