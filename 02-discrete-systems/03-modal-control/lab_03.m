clc
close all

num = [[0.1006, -0.2299, 1.758, 0.1326]];
det = [1, 0.5415, 8.667, 1.226];
system = tf(num,det);
zsystem_01 = c2d(system,0.1);
zsystem_1 = c2d(system,1);
zsystem_10 = c2d(system,10);
a =zero(zsystem_01)
W_01 = ss(zsystem_01);              %Описание в ПС системы с Т=0.1 с.           
uprav_01 = ctrb(W_01.A, W_01.B);    %Матрица управл. для Т=0.1 с.
rank_uprav_01 = rank(uprav_01)      %Ранг матрицы управл. системы с Т=0.1 с.
W_1 = ss(zsystem_1);
uprav_1 = ctrb(W_1.A, W_1.B);
rank_uprav_1 = rank(uprav_1)
W_10 = ss(zsystem_10)
uprav_10 = ctrb(W_10.A, W_10.B);
rank_uprav_10 = rank(uprav_10)
 mu = pi/log(0.3);      %Величина, зависящая от перерегулиров.
 betta = 2.5;           %Комплексаная чать.
 alfa = -betta/mu;      %Действительная часть
 %P = [complex(-alfa,betta),complex(-alfa,-betta),-17]  %Желаемые полюса
 P = [-1.1630, -0.8640, -0.2333]
 K_01 = acker(W_01.A, W_01.B,P)                        %Матрица коэффициентов Т = 0.1 с.
 betta = 1.4;           %Комплексаная чать.
 alfa = -betta/mu;      %Действительная часть
 %P = [complex(-alfa,betta),complex(-alfa,-betta),-0.4]  %Желаемые полюса
 P = [-2.2438, -0.2483, -0.0071]
 K_1 = acker(W_1.A, W_1.B,P)                           %Матрица коэффициентов Т = 1 с.
 betta = 0.16;           %Комплексаная чать.
 alfa = -betta/mu;      %Действительная часть
 %P = [complex(-alfa,betta),complex(-alfa,-betta),-0.00001]  %Желаемые полюса
 P = [-a(1), -a(2), a(3)]
 K_10 = acker(W_10.A, W_10.B,P/32)                        %Матрица коэффициентов Т = 10 с.
 % P = [-1.1630, -0.8640, -0.2333]
%26.4877   22.9348   29.6108 - 30(27)%
%8.6083    0.4502    1.7485 0%