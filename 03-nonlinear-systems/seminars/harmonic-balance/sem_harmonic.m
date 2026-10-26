clc
clear all

c = 1;
b = 1;
d = 15.5;

% Создание новой модели Simulink
model = 'sem_harmonic_22a_mdl';
open_system(model);

% Настройка параметров моделирования
% set_param('sem_harmonic_22a_mdl/Linear', 'y', '0');
% set_param('sem_harmonic_22a_mdl/Linear1', 'y1', '0');
set_param(model, 'Solver', 'ode15s');
set_param(model, 'StopTime', '5');

% Моделирование схемы
sim(model);

t = 0:0.01:10;

y = get_param(model, 'y');
u = get_param(model, 'u');
y1 = get_param(model, 'y1');
u1 = get_param(model, 'u1');

figure
subplot(2, 1, 1)
plot(t, y, t, u)
legend('Выход после ЛЧ', 'Выход после НЧ');

sublot(2, 1, 2)
plot(t, y1, t, u1)
legend('Выход после ЛЧ', 'Выход после НЧ');

