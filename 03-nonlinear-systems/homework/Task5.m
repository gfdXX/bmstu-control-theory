clc
close all
clear variables

% Параметры управления
a1 = 1;
k = 3;

x0 = [1; 0.5]; 
tspan = [0 10];

[t,x] = ode45(@sliding_mode, tspan, x0);
u = -k*sign(a1*x(:,1) + x(:,2));

% Построение графиков
figure;
subplot(2,1,1);
plot(t, x(:,1), t, x(:, 2));
legend("x1", "x2")
ylabel('x');
title('Изменение переменных состояния');
grid on;

subplot(2,1,2);
plot(t, u);
ylabel('u');
xlabel('Время [с]');
title('Изменение управляющего воздействия');
grid on;
%--------------%

[t,x] = ode45(@sliding_mode_saturation, tspan, x0);

s = a1*x(:,1) + x(:,2);
u = -k*sat(s);

% Построение графиков
figure;
subplot(2,1,1);
plot(t, x(:,1), t, x(:, 2));
legend("x_1", "x_2")
ylabel('x');
title('Изменение переменных состояния');
grid on;

subplot(2,1,2);
plot(t, u);
ylabel('u');
xlabel('Время [с]');
title('Изменение управляющего воздействия: функция насыщения \epsilon = 0.8');
grid on;
%--------------%

[t,x] = ode45(@sliding_mode_continuous_control, tspan, x0);

s = a1*x(:,1) + x(:,2);
u_continuous_control = -abs(x(:, 2)-sin(x(:, 1)).^2); 
u_rele = -k*sign(s);
u = u_continuous_control + u_rele;

% Построение графиков
figure;
subplot(2,1,1);
plot(t, x(:,1), t, x(:, 2));
legend("x_1", "x_2")
ylabel('x');
title('Изменение переменных состояния');
grid on;

subplot(2,1,2);
plot(t, u);
ylabel('u');
xlabel('Время [с]');
title('Изменение управляющего воздействия модификация: непрерывное управление');
grid on;

function dx = sliding_mode(t, x)
    dx = zeros(2, 1);
    k = 3;
    a1 = 1;

    dx(1) = x(2);
    dx(2) = 1/3*((-k*sign(a1*x(1) + x(2)) - 2*x(2) - sin(x(1))^2));
end

function dx = sliding_mode_saturation(t, x)
    dx = zeros(2, 1);
    k = 3;
    a1 = 1;

    dx(1) = x(2);
    dx(2) = 1/3*(-k*sat((a1*(x(1) + x(2)))) - 2*x(2) - sin(x(1))^2);
end

function dx = sliding_mode_continuous_control(t, x)
    dx = zeros(2, 1);
    k = 3;
    a1 = 1;

    dx(1) = x(2);
    dx(2) = 1/3*(-abs(x(2)-sin(x(1))^2)-k*sign(a1*(x(1) + x(2))) - 2*x(2) - sin(x(1))^2);
end

function u = sat(s)
    if abs(s/0.8) > 1
        u = sign(s/0.8);
    else
        u = s/0.8;
    end
end