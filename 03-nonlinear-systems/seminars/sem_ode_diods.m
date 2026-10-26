function dx = sem_ode_diods(t,x)

dx = zeros(2,1);

h = 17.76*x(1)-103.79*x(1)^2+229.62*x(1)^3-226.31*x(1)^4+83.72*x(1)^5;
% [phi, dphi]
dx(1) = 0.5*(-h+x(2));
dx(2) = 0.2*(-x(1)-1.5*x(2)+1.2);
end
