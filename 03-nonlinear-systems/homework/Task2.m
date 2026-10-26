clc
clear variables
close all force

T = 0:0.001:10;
X = zeros(length(T), 2);
X(1, :) = [pi+1 1];

for i = 1:length(T)-1
    u = 0;
    dt = T(i+1)-T(i);
    
    K1=dt*Koshi_nonlinear(T(i),X(i,:), u);
    K2=dt*Koshi_nonlinear(T(i)+dt/2, X(i,:)+K1'/2, u);
    K3=dt*Koshi_nonlinear(T(i)+dt/2, X(i,:)+K2'/2, u);
    K4=dt*Koshi_nonlinear(T(i)+dt, X(i,:)+K1', u);

    X(i+1,:) = X(i,:)+1/6*(K1'+2*K2'+2*K3'+K4');
end

figure(1)
subplot(2,1,1)
plot(T, X(:,1))
title('Переходный процесс нелинейной системы')
xlabel('Время, [c]');
ylabel('x1');
grid on;
subplot(2,1,2)
plot(T, X(:, 2))
title('Переходный процесс нелинейной системы')
xlabel('Время, [c]');
ylabel('x2');
grid on

T = 0:0.001:10;
X = zeros(length(T), 2);
X(1, :) = [pi+1 1];

for i = 1:length(T)-1
    u = 0;
    dt = T(i+1)-T(i);
    
    K1=dt*Koshi_linear(T(i),X(i,:), u);
    K2=dt*Koshi_linear(T(i)+dt/2, X(i,:)+K1'/2, u);
    K3=dt*Koshi_linear(T(i)+dt/2, X(i,:)+K2'/2, u);
    K4=dt*Koshi_linear(T(i)+dt, X(i,:)+K1', u);

    X(i+1,:) = X(i,:)+1/6*(K1'+2*K2'+2*K3'+K4');
end

figure(2)
subplot(2,1,1)
plot(T, X(:,1))
title('Переходный процесс линеаризованной системы')
xlabel('Время, [c]');
ylabel('x1');
grid on;
subplot(2,1,2)
plot(T, X(:, 2))
title('Переходный процесс линеаризованной системы')
xlabel('Время, [c]');
ylabel('x2');
grid on;

function dx = Koshi_nonlinear( t, x, u)
dx = zeros(2,1);

dx(1) = x(2);
dx(2) = (u - 2*x(2) - sin(x(1))^2) / 3;
end

function dx = Koshi_linear( t, x, u)
dx = zeros(2,1);

dx(1) = x(2);
dx(2) = -2/3*x(2) + u*1/3;
end