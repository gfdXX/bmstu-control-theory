clc 
clear all
close all

x = [1 2 4 9 23 8 12 4 2];
k = 2;          
nsigma = 2;    

y_custom = hampel_filter(x, k, nsigma);

y_builtin = hampel(x, k, nsigma);

disp('Результат фильтрации с помощью hampel_filter:');
disp(y_custom);
disp('Результат фильтрации с помощью встроенной функции hampel:');
disp(y_builtin);

figure;
plot(x, '-o'); hold on;
plot(y_custom, '-x'); 
plot(y_builtin, '-s'); 
legend('Исходный сигнал', 'hampel\_filter', 'hampel');
title('Сравнение фильтрации Хампеля');
xlabel('Индекс');
ylabel('Значение');

% Задание 2
f = 5;          
dt = 10^-3;        
t = 0:dt:0.5;

x = 2 * cos(2 * pi * f * t);

m = 5; 
a = -2;            
b = 2;             
x(randi(length(x), m, 1)) = a + (b - a) * rand(m, 1);

k_values = [1, 5, 10];
nsigma = 3;
y_filtered = cell(1, length(k_values)); 

for i = 1:length(k_values)
    k = k_values(i);
    y_filtered{i} = hampel_filter(x, k, nsigma);
end

figure;
plot(t, x, 'k-', 'DisplayName', 'Исходный сигнал'); hold on;
colors = {'r', 'g', 'b'};

for i = 1:length(k_values)
    plot(t, y_filtered{i}, colors{i}, 'DisplayName', sprintf('k = %d', k_values(i)));
end

legend;
title('Исследование влияния размера окна фильтра Хампеля');
xlabel('Время (с)');
ylabel('Амплитуда');
grid on;

% Задание 3
f = 5;            
dt = 10^-3;         
t = 0:dt:0.5;     

x = 2 * cos(2 * pi * f * t);

m = 5;           
a = -2;           
b = 2;            
x(randi(length(x), m, 1)) = a + (b - a) * rand(m, 1);

nsigma_values = [1, 5, 10];
k = 3;  
y_filtered = cell(1, length(nsigma_values));

for i = 1:length(nsigma_values)
    nsigma = nsigma_values(i);
    y_filtered{i} = hampel_filter(x, k, nsigma);
end

figure;
plot(t, x, 'k-', 'DisplayName', 'Исходный сигнал'); 
hold on;
colors = {'r', 'g', 'b'};

for i = 1:length(nsigma_values)
    plot(t, y_filtered{i}, colors{i}, 'DisplayName', sprintf('n_{\\sigma} = %d', nsigma_values(i)));
end

legend;
title('Исследование влияния числа стандартных отклонений в фильтре Хампеля');
xlabel('Время (с)');
ylabel('Амплитуда');
grid on;


function y = hampel_filter(x, k, nsigma)
    x_padded = padarray(x, [0 k], 'replicate');
    
    scale = -1 /(sqrt(2)*erfcinv(3/2)); % 1.4826
    
    y = x;
    
    for i = 1:length(x)
        window = x_padded(i:i + 2*k);
        
        mi = median(window);
        mad = scale * median(abs(window - mi));
        
        if abs(x(i) - mi) > nsigma * mad
            y(i) = mi;
        end
    end
end
