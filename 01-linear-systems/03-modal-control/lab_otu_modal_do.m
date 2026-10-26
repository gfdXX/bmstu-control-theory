% Создание передаточной функции с использованием zpk()
sys_zpk = zpk(1,[-3, -4, -5, -6], 100);
figure
step(sys_zpk);

sys_ss = ss(sys_zpk, 'minimal');

% Получение матриц A, B, C, D
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;
D = sys_ss.D;

F=place(A,B,[-9 -6 -7 -8]);
L=place(A',C', [-9 -6 -7 -8])';
A1=A-B*F;
sys_zpk2=ss(A1,B,C,D);
figure
step(sys_zpk2);
T=0.01;
cov_Input = 1e-6;
cov_Output = 1e-6;
set_param('lab_otu_observer/Step', 'SampleTime', 'T'); 
set_param('lab_otu_observer/Step', 'Time', '1.0'); 
set_param('lab_otu_observer/Step', 'Before', '0.0'); 
set_param('lab_otu_observer/Step', 'After', '1.0'); 
set_param('lab_otu_observer/Step', 'VectorParams1D', 'off');

set_param('lab_otu_observer/Входной шум', 'Ts', 'T');
set_param('lab_otu_observer/Входной шум', 'Cov', 'cov_Input');
set_param('lab_otu_observer/Измерительный шум', 'Ts', 'T');
set_param('lab_otu_observer/Измерительный шум', 'Cov', 'cov_Output');

set_param('lab_otu_observer/A/Constant', 'Value', 'A');
set_param('lab_otu_observer/B/Constant', 'Value', 'B');
set_param('lab_otu_observer/C/Constant', 'Value', 'C');
set_param('lab_otu_observer/D/Constant', 'Value', 'D');

set_param('lab_otu_observer/A1/Constant', 'Value', 'A');
set_param('lab_otu_observer/B1/Constant', 'Value', 'B');
set_param('lab_otu_observer/C1/Constant', 'Value', 'C');
set_param('lab_otu_observer/D1/Constant', 'Value', 'D');

set_param('lab_otu_observer/L/Constant', 'Value', 'L');
set_param('lab_otu_observer/F/Constant', 'Value', 'F');

DATA = sim('lab_otu_observer');

