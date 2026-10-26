clc
close all

t = 0:0.01:10;
x0 = [1; 0.5];
k1 = 2;

x0_obs = [0; 0];
dt = t(2)-t(1);

[T, XX] = ode45(@fuction_control, t, x0);

UU = -3*XX(:, 1) - 2*k1*XX(:, 1) + (sin(XX(:, 1))).^2 - 3*k1*(XX(:, 2) + k1*XX(:, 1)) + 3*k1^2*XX(:, 1);

figure;
subplot(3, 1, 1);
plot(T, XX(:, 1),  'DisplayName', 'Истинное состояние');
hold on;
grid on;

subplot(3, 1, 2);
plot(T, XX(:, 2),  'DisplayName', 'Истинное состояние');
hold on;
grid on;

subplot(3, 1, 3);
plot(T, UU,  'DisplayName', 'Истинное состояние');
hold on;
xlabel('Время [c]');
grid on;

e = [0.1; 0.05; 0.01];
y = XX(end, 1);

for n=1:3
    eps = e(n);

    U = zeros(length(t), 1);
    U(1) = 0;

    X = zeros(length(t), length(x0));
    X(1, :) = x0;

    X_obs = zeros(length(t), length(x0));
    X_obs(1, :) = x0_obs;
    for i=1:length(t)-1
        K1 = dt * function_observer(X_obs(i,:), y, eps);
        K2 = dt * function_observer(X_obs(i,:) + K1'/2, y, eps);
        K3 = dt * function_observer(X_obs(i,:) + K2'/2, y, eps);
        K4 = dt * function_observer(X_obs(i,:) + K3', y, eps);
        X_obs(i+1,:) = X_obs(i,:) + 1/6 * (K1' + 2*K2' + 2*K3' + K4');

        u_eq = -abs(-4*X_obs(i+1, 2) - 5*X_obs(i+1, 1)); 
        s = -3*X_obs(i+1, 1) - 2*k1*X_obs(i+1, 1) + sin(X_obs(i+1, 1))^2 - 3*k1*(X_obs(i+1, 2) + k1*X_obs(i+1, 1)) + 3*k1^2*X_obs(i+1, 1);
        u_rele = -sign(s);
        U(i+1) = u_eq + u_rele;

        K1 = dt * fuction_control_for_observer(X(i,:), U(i+1));
        K2 = dt * fuction_control_for_observer(X(i,:) + K1'/2, U(i+1));
        K3 = dt * fuction_control_for_observer(X(i,:) + K2'/2, U(i+1));
        K4 = dt * fuction_control_for_observer(X(i,:) + K3', U(i+1));
        X(i+1,:) = X(i,:) + 1/6 * (K1' + 2*K2' + 2*K3' + K4');

        y = X(i+1, 1);
    end

    subplot(3, 1, 1);
    plot(t, X(:, 1),  'DisplayName', ['Оценка состояния ε = ', num2str(eps)]);
    hold on
    ylabel("x1")
    title("Наблюдатель с большим коэффициентом усиления")

    subplot(3, 1, 2);
    plot(t, X(:, 2), 'DisplayName', ['Оценка состояния ε = ', num2str(eps)]);
    hold on
    ylabel("x2")

    subplot(3, 1, 3);
    plot(t, U, 'DisplayName', ['Оценка состояния ε = ', num2str(eps)]);
    hold on
    ylabel("u")
end

subplot(3, 1, 1);
legend;
subplot(3, 1, 2);
legend;
subplot(3, 1, 3);
legend;

function dx = fuction_control(t, x)
    dx = zeros(2, 1);
    k1 = 2;

    u = -3*x(1) - 2*k1*x(1) + sin(x(1))^2 - 3*k1*(x(2) + k1*x(1)) + 3*k1^2*x(1);

    dx(1) = x(2);
    dx(2) = 1/3*(u - 2*x(2) - sin(x(1))^2);
end

function dsdt = fuction_control_for_observer(x, u) 
    x1 = x(2);
    x2 = 1/3*(u - 2*x(2) - sin(x(1))^2);
    
    dsdt = [x1; x2];
end

function dX = function_observer(x, y, eps)
    x3 = x(2) + (2/eps) * (y - x(1));
    x4 = -7*x(1) - 8*x(2) + (1/eps^2) * (y - x(1));

    dX = [x3; x4];
end