clc
close all;

sys.m1 = 3;
sys.m2 = 0.2;
sys.m3 = 0.1;
sys.l = 0.3;
sys.b2 = 0.25;
sys.k2 = 0.25;
sys.F_sopr = 0;
sys.F_tr = 0;
F = @sem_01_func;
y0 = [0;0;pi/6;0.1];
t = 0:0.01:10;
t0 = 0;
t_end = 10;
h = 0.01;
result(:,1)=y0;
result_new(:,1)=y0;

for i=1:length(t)
    sys.F_upr = 0;
    M = [1, 0 ,0, 0; ...
        0, 1, 0, ((sys.m2+(sys.m3)/2)*cos(result(3,i))*sys.l)/(sys.m1 + sys.m2 + sys.m3);...
        0, 0, 1, 0;...
        0, ((sys.m2+(sys.m3)/2)*cos(result(3,i))*sys.l)/((sys.m2+(sys.m3)/3)*(sys.l)^2), 0, 1];
    Y = M*ode4(F, t0+(i-1)*h, h, t0+i*h, result(:,i),sys);
    result(:,i+1) = Y;
end

for i=1:length(t)
    sys.F_upr = 10*(0-result_new(4,i))+7*(0-result_new(3,i));
    M = [1, 0 ,0, 0; ...
        0, 1, 0, ((sys.m2+(sys.m3)/2)*cos(result_new(3,i))*sys.l)/(sys.m1 + sys.m2 + sys.m3);...
        0, 0, 1, 0;...
        0, ((sys.m2+(sys.m3)/2)*cos(result_new(3,i))*sys.l)/((sys.m2+(sys.m3)/3)*(sys.l)^2), 0, 1];
    Y = M*ode4(F, t0+(i-1)*h, h, t0+i*h, result_new(:,i),sys);
    result_new(:,i+1) = Y;
end

figure
subplot(2,1,1);
yyaxis left
plot(t,result(1,1:length(t)));
grid on;
ylabel('x, м');
yyaxis right
plot(t,result(2,1:length(t)));
grid on;
ylabel('x-speed, м/с');

subplot(2,1,2);
yyaxis left
plot(t,result(3,1:length(t))*180/pi);
grid on;
ylabel('fi, град.');
yyaxis right
plot(t,result(4,1:length(t))*180/pi);
grid on;
ylabel('fi-speed, град/с');

figure
subplot(2,1,1);
yyaxis left
plot(t,result_new(1,1:length(t)));
grid on;
ylabel('x, м');
yyaxis right
plot(t,result_new(2,1:length(t)));
grid on;
ylabel('x-speed, м/с');

subplot(2,1,2);
yyaxis left
plot(t,result_new(3,1:length(t))*180/pi);
grid on;
ylabel('fi, град.');
yyaxis right
plot(t,result_new(4,1:length(t))*180/pi);
grid on;
ylabel('fi-speed, град/с');

function dx = sem_01_func(t,x,sys)
dx = zeros(4,1);
dx(1) = x(2);
dx(2) = (sys.F_upr - sys.F_sopr - sys.F_tr - (sys.m2 + (sys.m3)/2)*(sys.l*(x(4)^2)*sin(x(3))))/(sys.m1+sys.m2+sys.m3);
dx(3) = x(4);
dx(4) = ((sys.m2+(sys.m3)/2)*9.81*sys.l*sin(x(3))-sys.b2*x(4)-sys.k2*x(3))/((sys.m2+(sys.m3)/3)*(sys.l)^2);
end

function yout = ode4(F,t0,h,tfinal,y0,sys)
     y = y0;
      yout = y;
      for t = t0 : h : tfinal-h
         s1 = F(t,y,sys);
         s2 = F(t+h/2, y+h*s1/2,sys);
         s3 = F(t+h/2, y+h*s2/2,sys);
         s4 = F(t+h, y+h*s3,sys);
         y = y + h*(s1 + 2*s2 + 2*s3 + s4)/6;
         yout = [y];
      end
end