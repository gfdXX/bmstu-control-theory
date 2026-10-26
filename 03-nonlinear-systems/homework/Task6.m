clc
close all
clear variables
close all force

x0 = [1;0.5];
T = 0:0.01:10;

X = zeros(length(T), 2);
U = zeros(length(T), 1);

X(1,:) = x0;


for i = 1:length(T)-1
    x = X(i,:);

    u = -3*x(1) - 2*2*x(1) + sin(x(1))^2 - 3*2*(x(2) + 2*x(1)) + 3*4*x(1);
    U(i) = u;

    dt=T(i+1)-T(i);

    K1=dt*back_stepping(T(i),x,u);
    K2=dt*back_stepping(T(i)+dt/2, x+K1'/2,u);
    K3=dt*back_stepping(T(i)+dt/2, x+K2'/2,u);
    K4=dt*back_stepping(T(i)+dt, x+K1',u);

    X(i+1,:) = X(i,:)+1/6*(K1'+2*K2'+2*K3'+K4');
end

figure;
subplot(2,1,1);
plot(T, X(:,1), T, X(:,2));
xlabel('Время [c]');
ylabel('Состояния');
legend('x_1', 'x_2');
title('Траектория состояний');
grid on

% Построение управления
subplot(2,1,2);
plot(T, U);
xlabel('Время [c]');
ylabel('u');
title('Управление');
grid on

function [dx] = back_stepping(t,x,u) 
    dx = zeros(2,1);
    
    dx(1) = x(2);
    dx(2) = -2*x(2)/3 - (sin(x(1))^2)/3 + u/3;
   end