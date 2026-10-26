close all, clear variables, clc
Gp = tf([0.1006 -0.2299 1.758 0.1326],[1 0.5415 8.667 1.226])
T = [1,2,4];
for i = 1:3
Gpd = c2d(Gp,T(i))
[num,den]= tfdata(Gpd);
num = cell2mat(num);
den = cell2mat(den);
q0 = 1/(num(2)+num(3)+num(4))
q1 = q0*den(2)
q2 = q0*den(3)
q3 = q0*den(4)
p1 = q0*num(2)
p2 = q0*num(3)
p3 = q0*num(4)
num = [q0 q1 q2 q3];
den = [1 -p1 -p2 -p3];
GR = tf(num, den, T(i), 'Variable', 'z^-1');
Gsys = Gpd*GR/(1+Gpd*GR);
Gn = 1/(1+GR*Gpd);
hold on
step(Gn)
grid on
xlim([0 10])
legend('To=1c','To=2c','To=4c')
% ltiview({'step', 'pzmap', 'bode', 'nyquist'}, Gsys)
end

% Программа для вычисления апериодического регулятора повышенного порядка в программной среде MATLAB.
Gpd = c2d(Gp,1);
[num,den]= tfdata(Gpd);
num = cell2mat(num);
den = cell2mat(den);

q0 = -18.6;
q1 = q0*(den(2)-1)+1/(num(2)+num(3)+num(4))
q2 = q0*(den(3)-den(2))+den(2)/(num(2)+num(3)+num(4))
q3 = q0*(den(4)-den(3))+den(3)/(num(2)+num(3)+num(4))
q4 = den(4)*(-q0+1/(num(2)+num(3)+num(4)))
p1 = q0*num(2)
p2 = q0*(num(3)-num(2))+num(2)/(num(2)+num(3)+num(4))
p3 = q0*(num(4)-num(3))+num(3)/(num(2)+num(3)+num(4))
p4 = -num(4)*(q0-1/(num(2)+num(3)+num(4)))

num = [q0 q1 q2 q3 q4];
den = [1 -p1 -p2 -p3 -p4];
GR = tf(num, den, 1, 'Variable', 'z^-1')

Gsys = Gpd*GR/(1+Gpd*GR);

Gn = 1/(1+GR*Gpd);
hold on
step(Gn)
grid on
xlim([0 10])
ltiview({'step', 'pzmap', 'bode', 'nyquist'}, Gsys)

Gpd = c2d(Gp,2);
[num,den]= tfdata(Gpd);
num = cell2mat(num);
den = cell2mat(den);

q0 = -10.6;
q1 = q0*(den(2)-1)+1/(num(2)+num(3)+num(4))
q2 = q0*(den(3)-den(2))+den(2)/(num(2)+num(3)+num(4))
q3 = q0*(den(4)-den(3))+den(3)/(num(2)+num(3)+num(4))
q4 = den(4)*(-q0+1/(num(2)+num(3)+num(4)))
p1 = q0*num(2)
p2 = q0*(num(3)-num(2))+num(2)/(num(2)+num(3)+num(4))
p3 = q0*(num(4)-num(3))+num(3)/(num(2)+num(3)+num(4))
p4 = -num(4)*(q0-1/(num(2)+num(3)+num(4)))

num = [q0 q1 q2 q3 q4];
den = [1 -p1 -p2 -p3 -p4];
GR = tf(num, den, 2, 'Variable', 'z^-1')

Gsys = Gpd*GR/(1+Gpd*GR);

Gn = 1/(1+GR*Gpd);
hold on
step(Gn)
ltiview({'step', 'pzmap', 'bode', 'nyquist'}, Gsys)

Gpd = c2d(Gp,4);
[num,den]= tfdata(Gpd);
num = cell2mat(num);
den = cell2mat(den);

q0 = -13.9;
q1 = q0*(den(2)-1)+1/(num(2)+num(3)+num(4))
q2 = q0*(den(3)-den(2))+den(2)/(num(2)+num(3)+num(4))
q3 = q0*(den(4)-den(3))+den(3)/(num(2)+num(3)+num(4))
q4 = den(4)*(-q0+1/(num(2)+num(3)+num(4)))
p1 = q0*num(2)
p2 = q0*(num(3)-num(2))+num(2)/(num(2)+num(3)+num(4))
p3 = q0*(num(4)-num(3))+num(3)/(num(2)+num(3)+num(4))
p4 = -num(4)*(q0-1/(num(2)+num(3)+num(4)))

num = [q0 q1 q2 q3 q4];
den = [1 -p1 -p2 -p3 -p4];
GR = tf(num, den, 4, 'Variable', 'z^-1')

Gsys = Gpd*GR/(1+Gpd*GR);

Gn = 1/(1+GR*Gpd);
hold on
step(Gn)
ltiview({'step', 'pzmap', 'bode', 'nyquist'}, Gsys)

legend('To=1c','To=2c','To=4c')
