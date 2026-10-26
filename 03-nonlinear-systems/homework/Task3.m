close all
clc

%  ---- Построение фазовых портретов для нелинейной системы — маятника ----

%   Построение фазовых портретов
initial_conditions = [
    2*pi-0.6, -0.8
    2*pi-0.6, -0.6
    2*pi-0.6, -0.4
    2*pi-0.6, -0.2

    pi+0.8, 0.8
    pi+0.6, 0.6
    pi+0.5, 0.5
    pi+0.4, 0.45
    pi+0.2, 0.4

    pi+0.8, -0.5
    pi+0.7, -0.45
    pi+0.6, -0.4

    pi+0.4, 0.6
    pi+0.2, 0.8
];

tspan = [0 10]; 

figure;
hold on;

for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [~, x] = ode45(@sem_ode_pendulum, tspan, x0);
    plot(x(:,1), x(:,2));
end

xlabel('x1');
ylabel('x2');
title('Фазовый портрет системы');
grid on;
hold off;

function dx = sem_ode_pendulum(t, x)
    u = 0;
    dx = zeros(2, 1);
    dx(1) = x(2);
    dx(2) = (u - 2*x(2) - sin(x(1))^2) / 3;
end
