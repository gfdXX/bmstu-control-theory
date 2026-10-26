clc
clear all
close all

% Параметры системы
Q = 1; % Вес состояния
R = 1; % Вес управления
T = 5; % Время моделирования
dt = 0.01; % Шаг времени

% Задание матриц A(t) и B(t) для линеаризованной системы
A = @(t) -1; % Производная состояния
B = @(t) 1; % Производная управления

% Численное решение уравнения Риккати
N = T / dt; % Количество шагов времени
t = linspace(0, T, N);
P = zeros(1, N); % Матрица Риккати
P(end) = 0; % Граничное условие P(∞) = 0
% Обратный расчет уравнения Риккати
for i = N-1:-1:1
    dP = -(A(t(i))' * P(i+1) + P(i+1) * A(t(i)) - P(i+1) * B(t(i)) * R^(-1) * B(t(i))' * P(i+1) + Q);
    P(i) = P(i+1) - dP * dt;
end

% Вычисление оптимального управления
x0 = 0.5; % Начальное состояние
x = zeros(1, N); x(1) = x0;
u_star = @(t) (1/8) * exp(-3*t) - (1/2) * exp(-t); % Оптимальное управление
u = zeros(1, N);
delta_u = zeros(1, N);

for i = 1:N-1
    delta_u(i) = -R^(-1) * B(t(i))' * P(i) * x(i);
    u(i) = u_star(t(i)) + delta_u(i);
    dx = A(t(i)) * x(i) + B(t(i)) * delta_u(i);
    x(i+1) = x(i) + dx * dt;
end

% Графики
figure;
subplot(3, 1, 1);
plot(t, x, 'LineWidth', 1.5);
title('Возмущение состояния \delta x(t)');
xlabel('Время (t)'); ylabel('\delta x(t)');
grid on;

subplot(3, 1, 2);
plot(t, delta_u, 'LineWidth', 1.5);
title('Возмущение управления \delta u(t)');
xlabel('Время (t)'); ylabel('\delta u(t)');
grid on;

subplot(3, 1, 3);
plot(t, u, 'LineWidth', 1.5);
title('Суммарное управление u^*(t) + \delta u(t)');
xlabel('Время (t)'); ylabel('u(t)');
grid on;