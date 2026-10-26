close all
          
wn = 2; 
T_vec = [3; 3];

dt = 0.1;
num_steps = 100; 
time_series = 0:dt:(num_steps-1)*dt;

A = [cos(wn*dt), sin(wn*dt)/wn; 
     -wn*sin(wn*dt), cos(wn*dt)];
B = [(1 - cos(wn*dt))/wn^2; 
      sin(wn*dt)/wn];

% Начальные условия
x0 = [1; 2];
x_dim = size(x0, 1);
u_dim = 1;

% Матрицы весов
Q = diag([15, 15]);
Sf = diag([10, 10]);
R = 2;

% Генерация последовательностей матриц
A_full = repmat(A, num_steps+1, 1);
B_full = repmat(B, num_steps+1, 1);
Q_full = repmat(Q, num_steps+1, 1);
R_full = repmat(R, num_steps+1, 1);

% Вычисление коэффициентов управления и моделирование
[K_seq, P] = compute_gains(A_full, B_full, Q_full, R_full, Sf, num_steps, x_dim, u_dim, T_vec);
[x_trajectory, u_trajectory] = simulate_system(A_full, B_full, K_seq, num_steps, x_dim, u_dim, dt, x0);

function [x_hist, u_hist] = simulate_system(A_seq, B_seq, K_seq, steps, n_dim, m_dim, dt, init_state)
    x = init_state;
    x_hist = zeros(n_dim, steps); 
    x_hist(:, 1) = init_state; 
    u_hist = zeros(m_dim, steps);
    t_vector = 0:dt:(steps-1)*dt;
    
    for k = 2:steps
        A_k = A_seq(k*n_dim+1:(k+1)*n_dim, :);
        B_k = B_seq(k*n_dim+1:(k+1)*n_dim, :);
        K_k = K_seq(:, k-1)';
        u_hist(:, k-1) = -K_k * x;
        x = A_k * x + B_k * u_hist(:, k-1); 
        x_hist(:, k) = x; 
    end
    
    figure;
    for i = 1:n_dim
        plot(t_vector, x_hist(i, :)) ; 
        hold on;
    end
    hold off;
    xlabel('Время, с');
    ylabel('x');
    legend('x_1', 'x_2');
    title('Изменение состояния системы');
    grid on;

    figure;
    for i = 1:m_dim
        plot(t_vector, u_hist(i, :));
        hold on;
    end
    xlabel('Время, с');
    ylabel('u');
    legend('u');
    title('Изменение управления');
    grid on;
end

function [K_hist, S_p] = compute_gains(A_seq, B_seq, Q_seq, R_seq, Sf, steps, n_dim, m_dim, T_vec)
    S_p = Sf; 
    K_hist = zeros(n_dim, steps-1); 
    
    for k = steps-1:-1:1
        A_k = A_seq(k*n_dim+1:(k+1)*n_dim, :);
        B_k = B_seq(k*n_dim+1:(k+1)*n_dim, :);
        R_k = R_seq(k*m_dim+1:(k+1)*m_dim, :);
        Q_k = Q_seq(k*n_dim+1:(k+1)*n_dim, :);
        
        A = A_k - B_k*R_k^(-1)*T_vec';
        Q = Q_k - T_vec*R_k^(-1)*T_vec';
        

        K = (R_k + B_k' * S_p * B_k)^(-1) * (B_k' * S_p * A +T_vec');
        
        K_hist(:, k) = K; 
        S_p = Q + A' * S_p * A - (A' * S_p * B_k) * K;
        
    end
end
