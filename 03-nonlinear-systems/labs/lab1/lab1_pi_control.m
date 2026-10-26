clc
close all

global x_zadannoe Ki_1 Kp_1 Ki_2 Kp_2 Ki_3 Kp_3 Ki_4 Kp_4 h e_int_1 e_int_2 e_int_3 e_int_4
e_int_1 = 0;
e_int_2 = 0;
e_int_3 = 0;
e_int_4 = 0;
Ki_1 = 8;
Kp_1 = 3;
Ki_2 = 3;
Kp_2 = 1.5;
Ki_3 = 3;
Kp_3 = 2;
Ki_4 = 2.5;
Kp_4 = 2;
h = 0.01;
x_zadannoe = [5*pi/6, 0, pi/3, 0];
m = 1;
l = 1;
mu = 0.5;
global u_fi u_tetta
x_0 = [pi/2,0.1,pi/2,0.1];
x_0_1 = [-pi/2,0,0,0];
model_2 = @sem_ode_2;
model_1 = @sem_ode_1;
t_0 = 0;
dt = 0.01;
t_end = 20;
t = 0:0.01:10;
x = zeros(4,length(t));
x(:,1) = x_0_1';

for i = 1:length(t)-1
    time = h*i:h:h*(i+1);
    err_1 = x_zadannoe(1)-x(1,i);
    err_2 = x_zadannoe(2)-x(2,i);
    err_3 = x_zadannoe(3)-x(3,i);
    err_4 = x_zadannoe(4)-x(4,i);
    e_int_1 = e_int_1+err_1*h;
    e_int_2 = e_int_2+err_2*h;
    e_int_3 = e_int_3+err_3*h;
    e_int_4 = e_int_4+err_4*h;
%     PI_1 = Kp_1*(err_1) + Ki_1*(e_int_1);
%     PI_2 = Kp_2*(err_2) + Ki_2*(e_int_2);
%     PI_3 = Kp_3*(err_3) + Ki_3*(e_int_3);
%     PI_4 = Kp_4*(err_4) + Ki_4*(e_int_4);
    wi_fi = 10*err_3+err_4*3+e_int_3*5+e_int_4*1;
    wi_tetta = 5*err_1 + 10*err_2+e_int_1*10+e_int_2*1;
    u_tetta = m*l*l*(-x(4,i)*x(4,i)*sin(x(1,i))*cos(x(1,i))-l*sin(x(1,i))/(m*9.81)+wi_tetta);
    u_fi = (m*l*l*(sin(x(1,i)))^2)*(2*x(2,i)*x(4,i)*cos(x(1,i))/sin(x(1,i))+wi_fi);
    [a,b] = ode15s(@(time,x) model_1(time,x),time,x(:,i));
    x(:,i+1) = b(length(b),:)';
end

figure
subplot(2,1,1)
title('\theta, d\theta');
yyaxis left
plot(t,x(1,:)*180/pi);
grid on;
ylabel('\theta');
yyaxis right
plot(t,x(2,:)*180/pi);
grid on;
ylabel('d\theta');

subplot(2,1,2)
title('\phi, d\phi');
yyaxis left
plot(t,x(3,:)*180/pi);
grid on;
ylabel('\phi');
yyaxis right
plot(t,x(4,:)*180/pi);
grid on;
ylabel('d\phi');


function dx = sem_ode_1 (t,x)
global u_fi u_tetta
dx = zeros(4,1);
m = 1;
l = 1;
mu = 0.5;
dx(1) = x(2);
dx(2) = u_tetta/(m*l*l) + x(4)*cos(x(1))*sin(x(1)) + m*9.81*sin(x(1))/l - mu*x(2)/(m*l*l);
dx(3) = x(4);
dx(4) = u_fi/(m*l*l*(sin(x(1))^2)) - 2*x(4)*x(2)*cos(x(1))/sin(x(1)) - mu*x(4)/(m*l*l*(sin(x(1))^2));
end

function dx = sem_ode_2 (t,x)
dx = zeros(4,1);
m = 1;
l = 1;
mu = 0.5;
dx(1) = x(2);
dx(2) = u_tetta/(m*l*l) + x(4)*cos(x(1))*sin(x(1)) + m*9.81*sin(x(1))/l - mu*x(2)/(m*l*l);
dx(3) = x(4);
dx(4) = u_fi/(m*l*l*(sin(x(1))^2)) - 2*x(4)*x(2)*cos(x(1))/sin(x(1)) - mu*x(4)/(m*l*l*(sin(x(1))^2));
end