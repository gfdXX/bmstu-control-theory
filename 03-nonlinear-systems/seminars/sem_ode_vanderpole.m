function dx = sem_ode_vanderpole(t,x)
mu = 0.5;
dx = zeros(2,1);

% [phi, dphi]
dx(1) = x(2);
dx(2) = -x(1)+mu*(1-x(1)^2)*x(2);