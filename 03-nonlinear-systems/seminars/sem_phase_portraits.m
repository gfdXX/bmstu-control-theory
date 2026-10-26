close all
clc

initial_conditions = [
    10, -8;
    6, -8;
    3.5, -8;
    -0.2, -8;
    -1.6, -8;
    -8, 7.1;
    -8, 8.9;
    -4.5, 10;
    -3.5, 10;
    0.5, 10
];
tspan = [0 10]; 

figure;
hold on;

for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [~, x] = ode45(@sem_ode_pendulum, tspan, x0);
    plot(x(:,1), x(:,2), 'LineWidth', 1.5);
end

xlim([min(initial_conditions(:,1)) max(initial_conditions(:,1))]);
ylim([min(initial_conditions(:,2)) max(initial_conditions(:,2))]);
xlabel('x1');
ylabel('x2');
title('Фазовый портрет системы');
grid on;
hold off;


x0 = [10, -8];
tspan = [0 10]; 
[t, x] = ode45(@sem_ode_pendulum, tspan, x0);

figure;
yyaxis left;
plot(t, x(:,1), 'b', 'LineWidth', 2);
ylabel('x1');
xlabel('t, [c]');
title('Зависимости x1(t), x2(t) для начальных условий (10,-8)');

yyaxis right;
plot(t, x(:,2), 'r', 'LineWidth', 2);
ylabel('x2');

legend('x1', 'x2');
grid on;


mu = 0.5;
initial_conditions = [
    4, 4;
    -4, 4;
    0.1, 0.1;
    -4, -4;
    4, -4
];

figure;
hold on;

for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [t, x] = ode45(@(t, x) vanderpol_oscillator(t, x, mu), [0 20], x0);
    
    plot(x(:,1), x(:,2), 'LineWidth', 1.5);
end

xlim([-5 5]);
ylim([-5 5]);
xlabel('x1');
ylabel('x2');
title('Фазовый портрет осциллятора Ван-дер-Поля');
grid on;
hold off;

%--
x0 = [4, 4];
mu = 0.5;
tspan = [0 20];
[t, x] = ode45(@(t, x) vanderpol_oscillator(t, x, mu), tspan, x0);

figure;
yyaxis left;
plot(t, x(:,1), 'b', 'LineWidth', 2);
ylabel('x1');
xlabel('t, [c]');
title('Зависимости x1(t), x2(t) для начальных условий (4,4)');

yyaxis right;
plot(t, x(:,2), 'r', 'LineWidth', 2);
ylabel('x2');

legend('x1', 'x2');
grid on;



initial_conditions = [
    -0.4, 0.2;
    -0.4, 1;
    -0.4, 1.25;
    -0.4, 1.58;
    1.6, 0.8;
    0.8, -0.4;
    0.78, -0.4;
    0.7, -0.4;
    1.6, -0.3
];

figure;
hold on;

for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [t, x] = ode45(@sem_ode_diods, [0 20], x0);
    
    plot(x(:,1), x(:,2), 'LineWidth', 1.5);
end

xlabel('x1');
ylabel('x2');
title('Фазовый портрет для системы Diods');
grid on;
hold off;


x0 = [-0.4, 0.2];
tspan = [0 20];
[t, x] = ode45(@sem_ode_diods, tspan, x0);

figure;
yyaxis left;
plot(t, x(:,1), 'b', 'LineWidth', 2);
ylabel('x1');
xlabel('t, [c]');
title('Зависимости x1(t), x2(t) для начальных условий (-0.4,0.2)');

yyaxis right;
plot(t, x(:,2), 'r', 'LineWidth', 2);
ylabel('x2');

legend('x1', 'x2');
grid on;

function dxdt = sem_ode_diods(t, x)
    function hx = h(x1)
        hx = 17.76*x1 - 103.79*x1^2 + 229.62*x1^3 - 226.31*x1^4 + 83.72*x1^5;
    end

    dxdt = zeros(2, 1);
    dxdt(1) = 0.5*(-h(x(1)) + x(2));
    dxdt(2) = 0.2*(-x(1) - 1.5*x(2) + 1.2);
end


function dxdt = vanderpol_oscillator(t, x, mu)
    dxdt = zeros(2, 1);
    dxdt(1) = x(2);
    dxdt(2) = -x(1) + mu * (1 - x(1)^2) * x(2);
end

function dxdt = sem_ode_pendulum(t, x)
    dxdt = zeros(2, 1);
    dxdt(1) = x(2);
    dxdt(2) = -10*sin(x(1)) - x(2);
end