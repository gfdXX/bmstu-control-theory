function dx = sem_ode_pendulum(t,x)

dx = zeros(2,1);

% [phi, dphi]
dx(1) = x(2);
dx(2) = -10*sin(x(1))-x(2);



