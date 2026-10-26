
    close all;
    clc;

    global g m l mu
    g=9.5;
    m=1;
    l=1;
    mu=0.5;


    % Начальные условия
    x_end = [5/6 * pi; 0; 0; 0];
    x0 = [-pi/2; 0; 0; 0];

    % Параметры управления и параметры для симуляции
    scenarios = {
        struct('k', [-5 -5 0.01 0.01; 500 -0.025 0.01 0.01], 'y', [0 0 0 0], 't', linspace(0, 20, 2000), 'ode_func', @lab_ode2),
        struct('k', [-2 -1 0.01 0.01; 2.25 1 0.01 0.01], 'y', [0; 0; 0; 0], 't', linspace(0, 15, 300), 'ode_func', @lab_ode_sm),
        struct('k', [-2.5 -0.9 0.01 0.01; 2.25 1 0.01 0.01], 'y', [0; 0; 0; 0], 't', linspace(0, 15, 400), 'ode_func', @lab_ode_sm2)
    };

    % Запуск сценариев
    for i = 1:length(scenarios)
        scenario = scenarios{i};
        [T, X] = ode15s(@(t, x) scenario.ode_func(t, x, scenario.k, x_end), scenario.t, x0);
        plot_results(T, X);
    end

function plot_results(T, X)
    figure;
    subplot(2, 1, 1);
    yyaxis left;
    plot(T, rad2deg(X(:, 1)));    
    yyaxis right;
    plot(T, rad2deg(X(:, 2)), 'r');    
    legend('Угол \theta', 'Угловая скорость \theta');    
    subplot(2, 1, 2);
    yyaxis left;
    plot(T, rad2deg(X(:, 3)));    
    yyaxis right;
    plot(T, rad2deg(X(:, 4)), 'r');    
    legend('Угол \phi', 'Угловая скорость \phi');
end

function dx = lab_ode(t, x, k, x_end)
    global g m l mu
    y = x_end - x;
    csum = zeros(1, 4);
    csum = csum + 0.001 * y';
    u_theta = sum(k(1, :) .* [y(1), y(2), csum(1), csum(2)]);
    u_phi = sum(k(2, :) .* [y(3), y(4), csum(3), csum(4)]);
    
    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = (1 / (m * l^2)) * u_theta + x(4)^2 * cos(x(1)) * sin(x(1)) + (m * g / l) * sin(x(1)) - (mu / (m * l^2)) * x(2);
    dx(3) = x(4);
    dx(4) = (1 / (m * l^2 * sin(x(1))^2)) * u_phi - 2 * x(2) * x(4) * cos(x(1)) / sin(x(1)) - (mu / (m * l^2 * sin(x(1))^2)) * x(4);
end

function dx = lab_ode2(t, x, k, x_end)
    global g m l mu
    y = x - x_end;
    s1 = sum(k(2, 1:2) .* y(1:2)');
    s2 = sum(k(2, 3:4) .* y(3:4)');
    u_theta = k(1, 1) * sign(s1);
    u_phi = k(1, 2) * sign(s2);

    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = (1 / (m * l^2)) * u_theta + x(4)^2 * cos(x(1)) * sin(x(1)) + (m * g / l) * sin(x(1)) - (mu / (m * l^2)) * x(2);
    dx(3) = x(4);
    dx(4) = (1 / (m * l^2 * sin(x(1))^2)) * u_phi - 2 * x(2) * x(4) * cos(x(1)) / sin(x(1)) - (mu / (m * l^2 * sin(x(1))^2)) * x(4);
end

function dx = lab_ode_sm(t, x, k, x_end)
    global g m l mu
    y = x - x_end;
    s1 = sum(k(2, 1:2) .* y(1:2)');
    s2 = sum(k(2, 3:4) .* y(3:4)');
    v_theta = k(1, 1) * sign(s1);
    v_phi = k(1, 2) * sign(s2);

    u_theta = m * l^2 * (-x(4)^2 * cos(x(1)) * sin(x(1)) - m * g / l * sin(x(1)) + v_theta);
    u_phi = m * l^2 * (sin(x(1))^2) * (2 * x(2) * x(4) * cos(x(1)) / sin(x(1)) + mu / (m * l^2 * sin(x(1))^2) * x(4) + v_phi);
    
    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = (1 / (m * l^2)) * u_theta + x(4)^2 * cos(x(1)) * sin(x(1)) + (m * g / l) * sin(x(1)) - (mu / (m * l^2)) * x(2);
    dx(3) = x(4);
    dx(4) = (1 / (m * l^2 * sin(x(1))^2)) * u_phi - 2 * x(2) * x(4) * cos(x(1)) / sin(x(1)) - (mu / (m * l^2 * sin(x(1))^2)) * x(4);
end

function dx = lab_ode_sm2(t, x, k, x_end)
    global g m l mu
    y = x - x_end;
    s1 = sum(k(2, 1:2) .* y(1:2)');
    s2 = sum(k(2, 3:4) .* y(3:4)');
    v_theta = k(1, 1) * sat(s1);
    v_phi = k(1, 2) * sat(s2);

    u_theta = m * l^2 * (-x(4)^2 * cos(x(1)) * sin(x(1)) - m * g / l * sin(x(1)) + v_theta);
    u_phi = m * l^2 * (sin(x(1))^2) * (2 * x(2) * x(4) * cos(x(1)) / sin(x(1)) + mu / (m * l^2 * sin(x(1))^2) * x(4) + v_phi);
    
    dx = zeros(4, 1);
    dx(1) = x(2);
    dx(2) = (1 / (m * l^2)) * u_theta + x(4)^2 * cos(x(1)) * sin(x(1)) + (m * g / l) * sin(x(1)) - (mu / (m * l^2)) * x(2);
    dx(3) = x(4);
    dx(4) = (1 / (m * l^2 * sin(x(1))^2)) * u_phi - 2 * x(2) * x(4) * cos(x(1)) / sin(x(1)) - (mu / (m * l^2 * sin(x(1))^2)) * x(4);
end

function y = sat(x)
    if abs(x) <= pi/3
        y = 300 * x;
    else
        y = sign(x);
    end
end
