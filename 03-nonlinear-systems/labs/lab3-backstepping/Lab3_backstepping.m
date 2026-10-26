clc
clear variables
close all force

x0 = [-pi/2; 0.1; pi/2; 0.1];
T = 0:0.01:20;

X = zeros(length(T), 4);
X(1,:) = x0;

global m l mu u_phi
m = 1;
l = 1;
mu = 0.5;
u_phi = 0;

for i = 1:length(T)-1
    x = X(i,:);

    u =  -x(2) - (m*l*l)/x(2) * (x(4)^2*x(2)*cos(x(1))*sin(x(1)) + m*9.81*x(2)*sin(x(1))/l);
    dt=T(i+1)-T(i);
    
    K1=dt*back_stepping(T(i),X(i,:), u);
    K2=dt*back_stepping(T(i)+dt/2, X(i,:)+K1'/2, u);
    K3=dt*back_stepping(T(i)+dt/2, X(i,:)+K2'/2, u);
    K4=dt*back_stepping(T(i)+dt, X(i,:)+K1', u);

    X(i+1,:) = X(i,:)+1/6*(K1'+2*K2'+2*K3'+K4');
end

figure
subplot(2, 1, 1);
yyaxis left;
plot(T, rad2deg(X(:, 1)));
hold on;
yyaxis right;
plot(T, (X(:, 2)), 'r');
hold off;
legend('Angle \theta', 'Angle speed \theta')
subplot(2, 1, 2);
yyaxis left;
plot(T, rad2deg(X(:, 3)));
yyaxis right;
hold on;
plot(T, (X(:, 4)), 'r');
hold off;
legend('Angle \phi', 'Angle speed \phi')

function [dx] = back_stepping(t,x, u_thetta) 
    global m l mu u_phi
    dx = zeros(4,1);
    dx(1) = x(2);
    dx(2) = u_thetta/(m*l*l) + x(4)^2*cos(x(1))*sin(x(1)) + m*9.81*sin(x(1))/l - mu*x(2)/(m*l*l);
    dx(3) = x(4);
    dx(4) = u_phi/(m*l*l*(sin(x(1))^2)) - 2*x(4)*x(2)*cos(x(1))/sin(x(1)) - mu*x(4)/(m*l*l*(sin(x(1))^2));
end