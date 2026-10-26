close all

omega_n = 1;
dt = 0.1;
n_steps = 100;
time_vector = 0:dt:(n_steps-1)*dt;
a = 2;
b = 5;

A_matrix = [1, dt; 
           dt*a^2, 1];
B_matrix = [
    dt^2 /2;
    b*dt];

% A_matrix = [cos(omega_n*dt), sin(omega_n*dt)/omega_n; 
%     -omega_n*sin(omega_n*dt), cos(omega_n*dt)];
% B_matrix = [(1 - cos(omega_n*dt))/omega_n^2; 
%      sin(omega_n*dt)/omega_n];

initial_state = [1; 2];
x_state = zeros(2, n_steps);
x_state(:, 1) = initial_state;

for idx = 2:n_steps
    x_state(:, idx) = A_matrix * x_state(:, idx-1);
end

figure;
plot(time_vector, x_state(1, :)); hold on;
plot(time_vector, x_state(2, :));
xlabel('Время, с');
ylabel('x');
legend('x_1', 'x_2');
grid on;
title("Динамика системы без управления")

q_weight_1 = 15; q_weight_2 = 15;
control_weight = 2;
final_condition_1 = 10; final_condition_2 = 10;

Q_matrix = diag([q_weight_1, q_weight_2]);
S_matrix = diag([final_condition_1, final_condition_2]);

P_matrix = S_matrix;
K_values = zeros(2, n_steps-1);

for idx = n_steps-1:-1:1
    K_gain = (control_weight + B_matrix' * P_matrix * B_matrix)^(-1) * (B_matrix' * P_matrix * A_matrix);
    K_values(:, idx) = K_gain;
    P_matrix = Q_matrix + A_matrix' * P_matrix * A_matrix - (A_matrix' * P_matrix * B_matrix) * K_gain;
end

x_optimal = initial_state;
x_optimal_state = zeros(2, n_steps);
x_optimal_state(:, 1) = initial_state;
u_control = zeros(1, n_steps-1);

for idx = 2:n_steps
    current_K = K_values(:, idx-1)';
    u_control(idx-1) = -current_K * x_optimal_state(:, idx-1);
    x_optimal_state(:, idx)  = A_matrix * x_optimal_state(:, idx-1) + B_matrix * u_control(idx-1);
end

figure;
plot(time_vector, x_optimal_state(1, :)); hold on;
plot(time_vector, x_optimal_state(2, :));
xlabel('Время, с');
ylabel('x');
legend('x_1', 'x_2');
title("Динамика с оптимальным управлением")
grid on;

figure;
plot(time_vector(1:end-1), u_control);
xlabel('Время, с');
ylabel('u');
legend('u');
title("Управление")
grid on;

