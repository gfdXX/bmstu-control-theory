clc
close all
sys.u = 0;
sys.e = 0;
F = @M;
N = @Nabl;

t = 0:0.01:10;
h = 0.01;
t0 = 0;
t_end = 10;
x1 = zeros(3,length(t));
x1(1:3,1) = [1,1,1];

x2 = zeros(3,length(t));
x2(1:3,1) = [1,1,1];
x_n2 = zeros(2,length(t));
x_n2(:,1) = [0,0];

x3 = zeros(3,length(t));
x3(1:3,1) = [1,1,1];
x_n3 = zeros(2,length(t));
x_n3(:,1) = [0,0];

x4 = zeros(3,length(t));
x4(1:3,1) = [1,1,1];
x_n4 = zeros(2,length(t));
x_n4(:,1) = [0,0];

U1(1) = 0;
U2(1) = 0;
U3(1) = 0;
U4(1) = 0;

for i = 1:length(t)-1
    sys.u = -x1(2,i)^3 - x1(1,i) - x1(2,i);
    U1(i+1) = sys.u;
    x1(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x1(1:2,i),sys,0);
    x1(3,i+1) = x1(1,i+1);
end

for i = 1:length(t)-1
    sys.e = 0.1;
    x_n2(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,x2(3,i));
    sys.u = -x_n2(2,i+1)^3 - x_n2(1,i+1) - x_n2(2,i+1);
    U2(i+1) = sys.u;
    x2(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,0);
    x2(3,i+1) = x2(1,i+1);
end

for i = 1:length(t)-1
    sys.e = 0.01;
    x_n3(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x3(1:2,i),sys,x3(3,i));
    sys.u = -x_n3(2,i+1)^3 - x_n3(1,i+1) - x_n3(2,i+1);
    U3(i+1) = sys.u;
    x3(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x3(1:2,i),sys,0);
    x3(3,i+1) = x3(1,i+1);
end

for i = 1:length(t)-1
    sys.e = 0.005;
    x_n4(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x4(1:2,i),sys,x4(3,i));
    sys.u = -x_n4(2,i+1)^3 - x_n4(1,i+1) - x_n4(2,i+1);
    U4(i+1) = sys.u;
    x4(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x4(1:2,i),sys,0);
    x4(3,i+1) = x4(1,i+1);
end

figure
subplot(3,1,1);
plot(t,x1(1,1:length(t)),t,x2(1,1:length(t)),t,x3(1,1:length(t)),t,x4(1,1:length(t)));
title('Обычное управление')
xlabel('t, c');
ylabel('x1');
grid on;
legend('без набл.','\epsilon = 0.1','\epsilon = 0.01','\epsilon = 0.005');
subplot(3,1,2);
plot(t,x1(2,1:length(t)),t,x2(2,1:length(t)),t,x3(2,1:length(t)),t,x4(2,1:length(t)));
title('Обычное управление')
xlabel('t, c');
ylabel('x2');
grid on;
legend('без набл.','\epsilon = 0.1','\epsilon = 0.01','\epsilon = 0.005');

subplot(3,1,3);
plot(t,U1(1,1:length(t)),t,U2(1,1:length(t)),t,U3(1,1:length(t)),t,U4(1,1:length(t)));
title('Обычное управление')
xlabel('t, c');
ylabel('F_у_п_р');
grid on;
legend('без набл.','\epsilon = 0.1','\epsilon = 0.01','\epsilon = 0.005');

for i = 1:length(t)-1
    sys.u = SAT(-x1(2,i)^3 - x1(1,i) - x1(2,i),5);
    U1(i+1) = sys.u;
    x1(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x1(1:2,i),sys,0);
    x1(3,i+1) = x1(1,i+1);
end

for i = 1:length(t)-1
    sys.e = 0.1;
    x_n2(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,x2(3,i));
    sys.u = SAT(-x_n2(2,i+1)^3 - x_n2(1,i+1) - x_n2(2,i+1),5);
    U2(i+1) = sys.u;
    x2(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,0);
    x2(3,i+1) = x2(1,i+1);
end

for i = 1:length(t)-1
    sys.e = 0.01;
    x_n3(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x3(1:2,i),sys,x3(3,i));
    sys.u = SAT(-x_n3(2,i+1)^3 - x_n3(1,i+1) - x_n3(2,i+1),5);
    U3(i+1) = sys.u;
    x3(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x3(1:2,i),sys,0);
    x3(3,i+1) = x3(1,i+1);
end

for i = 1:length(t)-1
    sys.e = 0.005;
    x_n4(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x4(1:2,i),sys,x4(3,i));
    sys.u = SAT(-x_n4(2,i+1)^3 - x_n4(1,i+1) - x_n4(2,i+1),5);
    U4(i+1) = sys.u;
    x4(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x4(1:2,i),sys,0);
    x4(3,i+1) = x4(1,i+1);
end

figure
subplot(3,1,1);
plot(t,x1(1,1:length(t)),t,x2(1,1:length(t)),t,x3(1,1:length(t)),t,x4(1,1:length(t)));
title('Управление с насыщением')
xlabel('t, c');
ylabel('x1');
grid on;
legend('без набл.','\epsilon = 0.1','\epsilon = 0.01','\epsilon = 0.005');
subplot(3,1,2);
plot(t,x1(2,1:length(t)),t,x2(2,1:length(t)),t,x3(2,1:length(t)),t,x4(2,1:length(t)));
title('Управление с насыщением')
xlabel('t, c');
ylabel('x2');
grid on;
legend('без набл.','\epsilon = 0.1','\epsilon = 0.01','\epsilon = 0.005');

subplot(3,1,3);
plot(t,U1(1,1:length(t)),t,U2(1,1:length(t)),t,U3(1,1:length(t)),t,U4(1,1:length(t)));
title('Управление с насыщением')
xlabel('t, c');
ylabel('F_у_п_р');
grid on;
legend('без набл.','\epsilon = 0.1','\epsilon = 0.01','\epsilon = 0.005');

figure
plot(x2(1,:),x2(2,:));
hold on;
plot(x2(1,1),x2(2,1),'*'); %начало
plot(x2(1,end),x2(2,end),'x');%конец
grid on;
xlabel('x1');
ylabel('x2');
title('Фазовый портрет');
for j = 1:6 %было 20
x2 = zeros(3,length(t));
x2(1:3,1) = [rand(1),rand(1),rand(1)];
x_n2 = zeros(2,length(t));
x_n2(:,1) = x2(1:2,1);
for i = 1:length(t)-1
    sys.e = 0.1;
    x_n2(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,x2(3,i));
    sys.u = SAT(-x_n2(2,i+1)^3 - x_n2(1,i+1) - x_n2(2,i+1),5);
    U2(i+1) = sys.u;
    x2(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,0);
    x2(3,i+1) = x2(1,i+1);
end
plot(x2(1,:),x2(2,:));
plot(x2(1,1),x2(2,1),'*');
plot(x2(1,end),x2(2,end),'x');
end
% for j = 1:2
% x2 = zeros(3,length(t));
% x2(1:3,1) = [-rand(1),-rand(1),-rand(1)];
% x_n2 = zeros(2,length(t));
% x_n2(:,1) = x2(1:2,1);
% for i = 1:length(t)-1
%     sys.e = 0.1;
%     x_n2(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,x2(3,i));
%     sys.u = SAT(-x_n2(2,i+1)^3 - x_n2(1,i+1) - x_n2(2,i+1),5);
%     U2(i+1) = sys.u;
%     x2(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,0);
%     x2(3,i+1) = x2(1,i+1);
% end
% plot(x2(1,:),x2(2,:));
% plot(x2(1,1),x2(2,1),'*');
% plot(x2(1,end),x2(2,end),'x');
% end
% for j = 1:2
% x2 = zeros(3,length(t));
% x2(1:3,1) = [-rand(1),rand(1),rand(1)];
% x_n2 = zeros(2,length(t));
% x_n2(:,1) = x2(1:2,1);
% for i = 1:length(t)-1
%     sys.e = 0.1;
%     x_n2(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,x2(3,i));
%     sys.u = SAT(-x_n2(2,i+1)^3 - x_n2(1,i+1) - x_n2(2,i+1),5);
%     U2(i+1) = sys.u;
%     x2(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,0);
%     x2(3,i+1) = x2(1,i+1);
% end
% plot(x2(1,:),x2(2,:));
% plot(x2(1,1),x2(2,1),'*');
% plot(x2(1,end),x2(2,end),'x');
% end
% for j = 1:2
% x2 = zeros(3,length(t));
% x2(1:3,1) = [rand(1),-rand(1),-rand(1)];
% x_n2 = zeros(2,length(t));
% x_n2(:,1) = x2(1:2,1);
% for i = 1:length(t)-1
%     sys.e = 0.1;
%     x_n2(:,i+1) = ode4(N, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,x2(3,i));
%     sys.u = SAT(-x_n2(2,i+1)^3 - x_n2(1,i+1) - x_n2(2,i+1),5);
%     U2(i+1) = sys.u;
%     x2(1:2,i+1) = ode4(F, t0+(i-1)*h, h, t0+i*h, x2(1:2,i),sys,0);
%     x2(3,i+1) = x2(1,i+1);
% end
% plot(x2(1,:),x2(2,:));
% plot(x2(1,1),x2(2,1),'*');
% plot(x2(1,end),x2(2,end),'x');
% end
hold off;


function mistake = SAT(mistake, predel) %%Насыщение
    if (abs(mistake) > predel)
        mistake = predel* sign(mistake);
    end
    if (abs(mistake))<0.00
        mistake = 0;
    end
end

function dx = M(t,x,sys,y)
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = (sys.u - 2*x(2) - sin(x(1))^2) / 3;
end
function dx = Nabl(t,x,sys,y)
    dx = zeros(2,1);
    dx(1) = -x(2) + (2/sys.e)*(y-x(1));
    dx(2) = (1/(sys.e)^2)*(y-x(1));
end
function yout = ode4(F,t0,h,tfinal,y0,sys,ya)
     y = y0;
      yout = y;
      for t = t0 : h : tfinal-h
         s1 = F(t,y,sys,ya);
         s2 = F(t+h/2, y+h*s1/2,sys,ya);
         s3 = F(t+h/2, y+h*s2/2,sys,ya);
         s4 = F(t+h, y+h*s3,sys,ya);
         y = y + h*(s1 + 2*s2 + 2*s3 + s4)/6;
         yout = [y];
      end
end