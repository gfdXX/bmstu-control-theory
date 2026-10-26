clc
clear variables
close all force

c = 1;
b = 1;
d = 15.5;
% x1 = linspace(-2, 2, 200);
% y1 = rele(x1, c, b);
% x2 = linspace(2, -2, 200);
% y2 = rele(x2, c, b);
% plot(x1, y1, x2, y2)

DATA = sim('sem_harmonic_mdl', 'SaveOutput', 'on', 'SaveFormat', 'Dataset');

t = DATA.tout;
fd = 1/ ((t(length(t))-t(1))/length(t));

% s = DATA.yout{9}.Values.Data;
% figure
% plot(t, s)
% s = s-mean(s);
% s = s .* tukeywin(length(s),0.05);


% S = abs(fft(s)/length(s));
% range = 0:1:length(s)-1;
% f = fd * range / length(s);
% figure
% plot(f,S)


% set_param("sem_harmonic_mdl", 'Solver', 'ode15s');
% set_param("sem_harmonic_mdl", 'SolverType', 'Variable-step')


figure
subplot(2, 1, 1)
plot(DATA.tout, DATA.yout{2}.Values.Data, DATA.tout, DATA.yout{1}.Values.Data)
subplot(2, 1, 2)
plot(DATA.tout, DATA.yout{4}.Values.Data, DATA.tout, DATA.yout{3}.Values.Data)

figure
subplot(2, 1, 1)
plot(DATA.tout, DATA.yout{6}.Values.Data, DATA.tout, DATA.yout{5}.Values.Data)
legend("nonlinear", "linear part");
ylabel('amplitude') 
xlabel('time') 
subplot(2, 1, 2)
plot(DATA.tout, DATA.yout{7}.Values.Data, DATA.tout, DATA.yout{8}.Values.Data)
legend("after linearization", "linear part");
ylabel('amplitude') 
xlabel('time') 


% Y = DATA.yout{1}.Values.Data;
% X = DATA.yout{2}.Values.Data;
% Y_est = DATA.yout{3}.Values.Data;
% X_est = DATA.yout{4}.Values.Data;
% Y_err = DATA.yout{5}.Values.Data;