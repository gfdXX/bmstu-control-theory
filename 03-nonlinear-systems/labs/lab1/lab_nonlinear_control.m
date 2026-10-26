close all;
clc;

global g m l mu csum k x_end
g = 9.8;
m = 1;
l = 1;
mu = 0.5;

k = [ 20 10 0.01 0.01; 20 10 0.01 0.01]; %%Коэффициенты
csum = [0 0 0 0];           %%Кумулятивная ошибки

x_end = [   5/6*pi  ;
            0       ;
            pi/3    ;
            0      ];
x0 = [-pi/2 ;
        0   ;
        0   ;
        0  ];


t=[0 0.01];
for i=0:2000
    [~, X] = ode15s(@lab_ode, t,  x0);
    x0=X(end,:);
    t(1,1)=t(1,1)+0.01;
    t(1,2)=t(1,2)+0.01;

end
figure
subplot(2, 1, 1);
yyaxis left;
plot(T, rad2deg(X(:, 1)));
hold on;
yyaxis right;
plot(T, rad2deg(X(:, 2)), 'r');
hold off;
legend('Angle \theta', 'Angle speed \theta')
subplot(2, 1, 2);
yyaxis left;
plot(T, rad2deg(X(:, 3)));
yyaxis right;
hold on;
plot(T, rad2deg(X(:, 4)), 'r');
hold off;
legend('Angle \phi', 'Angle speed \phi')


csum = [0 0 0 0];

t = linspace(0, 3, 300);
[T, X] = ode15s(@lab_ode2, t, x0);

figure
subplot(2, 1, 1);
yyaxis left;
plot(T, rad2deg(X(:, 1)));
hold on;
yyaxis right;
plot(T, rad2deg(X(:, 2)), 'r');
hold off;
legend('Angle \theta', 'Angle speed \theta')
subplot(2, 1, 2);
yyaxis left;
plot(T, rad2deg(X(:, 3)));
yyaxis right;
hold on;
plot(T, rad2deg(X(:, 4)), 'r');
hold off;
legend('Angle \phi', 'Angle speed \phi')

function [dx] = lab_ode(t, x)
global g m l mu csum k x_end
    y = x_end - x; %%Вычисляется разница между желаемым конечным состоянием x_end и текущим состоянием
    csum(1) = csum(1) + 0.001*y(1);
    csum(2) = csum(2) + 0.001*y(2);
    csum(3) = csum(3) + 0.001*y(3);
    csum(4) = csum(4) + 0.001*y(4);

    %%Управляющие воздействия 
    u_theta = (k(1, 1)*y(1) + k(1, 2)*y(2) + k(1, 3)*csum(1) + k(1, 4)*csum(2));
    u_phi = (k(2, 1)*y(3) + k(2, 2)*y(4) + k(2, 3)*csum(3) + k(2, 4)*csum(4));

    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = 1/(m*l^2)*u_theta + x(4)^2*cos(x(1))*sin(x(1)) + m*g/l*sin(x(1)) - mu/(m*l^2)*x(2);
    dx(3) = x(4);
    dx(4) = 1/(m*l^2*(sin(x(1))^2))*u_phi - 2*x(2)*x(4)*cos(x(1))/sin(x(1)) - mu/(m*l^2*(sin(x(1))^2))*x(4);
end

function [dx] = lab_ode2(t, x)
global g m l mu csum k x_end
    y = x_end - x;
    csum(1) = csum(1) + 0.001*y(1);
    csum(2) = csum(2) + 0.001*y(2);
    csum(3) = csum(3) + 0.001*y(3);
    csum(4) = csum(4) + 0.001*y(4);
    
    %%Промежуточные значения, вычисление ошибки
    v_theta = (k(1, 1)*y(1) + k(1, 2)*y(2) + k(1, 3)*csum(1) + k(1, 4)*csum(2));
    v_phi = (k(2, 1)*y(3) + k(2, 2)*y(4) + k(2, 3)*csum(3) + k(2, 4)*csum(4));

    %%Управляющие воздействия
    u_theta = m*l^2*(-x(4)^2*cos(x(1))*sin(x(1)) - m*g/l*sin(x(1)) + v_theta);
    u_phi = (m*l^2*(sin(x(1))^2))*(2*x(2)*x(4)*cos(x(1))/sin(x(1)) + mu/(m*l^2*(sin(x(1))^2))*x(4) + v_phi);

    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = 1/(m*l^2)*u_theta + x(4)^2*cos(x(1))*sin(x(1)) + m*g/l*sin(x(1)) - mu/(m*l^2)*x(2);
    dx(3) = x(4);
    dx(4) = 1/(m*l^2*(sin(x(1))^2))*u_phi - 2*x(2)*x(4)*cos(x(1))/sin(x(1)) - mu/(m*l^2*(sin(x(1))^2))*x(4);
end