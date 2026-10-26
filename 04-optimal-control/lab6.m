    clc; 
    clear; 
    close all;

    T = 10;  
    N = 500;
    tGrid = linspace(0,T,N);

    x_star = @(t) 0.5*(exp(-t));
    u_star = @(t) (1/8)*exp(-3*t) - (1/2)*exp(-t);
    lam_star = @(t) 0;

    f_x = @(t) -3*( x_star(t)^2 );   % df/dx = -3 x^2
    f_u = @(t) 1;                    % df/du = 1

    H_xx = @(t) localHxx(x_star(t), u_star(t), lam_star(t));
    H_uu = @(t) 1;                   % d^2(H)/du^2 = 1
    H_ux = @(t) 0;                   % d^2(H)/du dx

    tRev = linspace(T,0,N);
    
    S_T = 0;
    V_T = 0;
    P_T = 0;
    end_cond = [S_T; V_T; P_T];

    % Решаем "назад" во времени:
    [trev, solRev] = ode45(@(tau, SVP) ...
        dynS_V_P(tau, SVP, f_x, f_u, H_xx, H_uu), ...
        tRev, end_cond);

    % Разворачиваем, чтобы получить S(t),V(t),P(t) на [0,T]
    tSol = flipud(trev);
    sol = flipud(solRev);
    S_sol = sol(:,1);
    V_sol = sol(:,2);
    P_sol = sol(:,3);
    S_func = @(tq) interp1(tSol, S_sol, tq, 'spline');
    V_func = @(tq) interp1(tSol, V_sol, tq, 'spline');
    P_func = @(tq) interp1(tSol, P_sol, tq, 'spline');

    %--- Далее получаем K(t) = H_uu^{-1} [H_ux + f_u^T S] = S(t), т.к. H_ux=0, f_u=1, H_uu=1
    K_func = @(tq) S_func(tq);

    x0 = 0.5;
    dx0 = 2;
    ode_delta_x = @(t,dx) f_x(t)*dx + f_u(t)*( -(K_func(t) - H_uu(t) * f_u(t)*V_func(t)*P_func(t) * V_func(t)) * dx );
    [t_dx, dx_sol] = ode45(ode_delta_x, tGrid, dx0);
    
    n=length(tGrid);
    m=length(x0);

    T = tGrid;
    X=zeros(n,m);
    X(1,:)=x0 + dx0;

    for i=1:length(tGrid)-1
        dt=T(i+1)-T(i);
        U_dx = -(K_func(T(i)) - H_uu(T(i)) * f_u(T(i))*V_func(T(i))*P_func(T(i)) * V_func(T(i))) * dx_sol(i);
        U_result = u_star(T(i)) + U_dx;

        K1=dt*dz_func(T(i),X(i,:), U_result);
        K2=dt*dz_func(T(i)+dt/2, X(i,:)+K1'/2, U_result);
        K3=dt*dz_func(T(i)+dt/2, X(i,:)+K2'/2, U_result);
        K4=dt*dz_func(T(i)+dt, X(i,:)+K3', U_result);

        X(i+1,:) = X(i,:)+1/6*(K1'+2*K2'+2*K3'+K4');
    end
    
    t_res = tGrid;
    dynamic_with_du = X;

    % Считаем delta u(t):
    delta_u = zeros(size(dx_sol));
    for k=1:length(t_dx)
       tk = t_dx(k);
       delta_u(k) = -K_func(tk)* dx_sol(k);
    end
    
    figure; 
    plot(t_dx, dx_sol);
    xlabel('Время, с'); 
    ylabel('\deltax(t)');
    grid on;
    title('Возмущение состояния \deltax(t)');

    figure; 
    plot(t_dx, delta_u, 'DisplayName','\deltau(t)');
    grid on;
    xlabel('Время, c');  
    ylabel('\deltau(t)');
    title('Возмущение управления \deltau(t)');

%     figure; hold on; grid on;
%     plot(tSol, S_sol,'r','LineWidth',1.5, 'DisplayName','S(t)');
%     plot(tSol, V_sol,'b','LineWidth',1.5, 'DisplayName','V(t)');
%     plot(tSol, P_sol,'m','LineWidth',1.5, 'DisplayName','P(t)');
%     xlabel('t'); ylabel('value');
%     legend('Location','best'); title('Solutions for S, V, P (backward ODE)');
% 
%     disp('Done. Plots show S(t), V(t), P(t), and the resulting \delta x(t), \delta u(t).');

    figure
    hold on
    plot(t_dx, arrayfun(x_star, t_dx));
    plot(t_res, dynamic_with_du)
    xlabel('Время, c');  
    ylabel('Состояние x(t)');
    hold off
    grid on
    legend('Оптимальная траектория x^*(t)', 'Динамика с возмущением \deltau(t)');
    title('Сравнение траекторий состояния x(t)');

    function dSVP = dynS_V_P(tau, SVP, fx_func, fu_func, Hxx_func, Huu_func)
        Sval = SVP(1);
        Vval = SVP(2);
        Pval = SVP(3);

        fx_now = fx_func(tau);
        fu_now = fu_func(tau);
        Hxx_now = Hxx_func(tau);
        Huu_now = Huu_func(tau);

        K_now = Sval; 
        
        S_dot = - ( fx_now*Sval + Sval*fx_now - K_now^2 +  Hxx_now );

        V_dot = - ( fx_now - fu_now*K_now ) * Vval;
        
        P_dot = (fu_now^2 / Huu_now) * (Vval^2);

        dSVP = [S_dot; V_dot; P_dot];
    end

    function val = localHxx(x, u, lam)
        val = 1 - 6*lam*x;
    end

    function dx = dz_func(t, x, u)
        dx = -x^3 + u;
    end
