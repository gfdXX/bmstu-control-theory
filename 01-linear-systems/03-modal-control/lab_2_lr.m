% clc
% close all
% 
% system_1 = zpk(-1,[-3, -4, -5 ,-6], 100);
% S_1 = ss(system_1,'minimal');
% figure(1)
% step(system_1);
% hold on
% F = place(S_1.A,S_1.B,[-9, -6, -7, -8]);
% Acl = S_1.A-S_1.B*F;
% system_1_mode = ss(Acl,S_1.B,S_1.C,S_1.D);
% step(system_1_mode);
% hold off
% sobstv_num = eigs(S_1.A-S_1.B.*eye(4,4));
% system_1_one_sviaz = ss(S_1.A,sobstv_num,S_1.C,S_1.D);
% figure(2)
% step(system_1_one_sviaz);
% F_05 = place(S_1.A,S_1.B,0.5*[-9, -6, -7, -8]);
% Acl = S_1.A-S_1.B*F_05;
% system_1_mode05 = ss(Acl,S_1.B,S_1.C,S_1.D);
% figure(3)
% step(system_1_mode);
% hold on
% step(system_1_mode05);
% 
% F_2 = place(S_1.A,S_1.B,2*[-9, -6, -7, -8]);
% Acl = S_1.A-S_1.B*F_2;
% system_1_mode2 = ss(Acl,S_1.B,S_1.C,S_1.D);
% step(system_1_mode2);
% hold off
% legend('P','0.5P','2P')
% 
sys = tf([4 3 1], [1 4 1 1 3]);
disp(sys)
rlocus(sys)