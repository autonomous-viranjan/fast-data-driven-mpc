function J = CostFunction(design_vector,s0,v0,tf)

c00 = 0.267;
c10 = 0.014;
c01 = 0.184;
c20 = 0.001;
c11 = 0.048;
c02 = 0.224;
Fd = 0.1;
% Parameters
M = 1200;
Av = 2.5;
Cd = 0.32;
rho = 1.184;
mu = 0.013;
g = 9.81;

u(1:tf-1,1) = design_vector(1:tf-1,1);
u(tf,1) = 0;
s(1,1) = s0;
s(2:tf,1) = design_vector(tf:2*tf-2,1);
v(1,1) = v0;
v(2:tf,1) = design_vector(2*tf-1:3*tf-3,1);

a = (-1/(2*M))*Cd*rho*Av*v.^2 - mu*g + u;

fuel = c00*ones(length(v),1) + c10*v + c01*a + c20*v.^2 + c11*(v.*a) + c02*(a.^2);

Fuel = zeros(tf,1);
for i = 1:tf
    if v(i) == 0 || u(i) < 0
        xi = 1;
    else
        xi = 0;
    end
    
    Fuel(i) = (1-xi)*(fuel(i)) + xi*Fd;
end

J = 0;
for i = 1:tf
    J = J + (Fuel(i)/v(i));
end


end