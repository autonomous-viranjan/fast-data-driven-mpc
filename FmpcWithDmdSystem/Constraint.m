function [c,ceq] = Constraint(design_vector,x_lead,s0,v0,df,Ts,dmin,alpha,tconstraint)

% Parameters
M = 1200;
Av = 2.5;
Cd = 0.32;
rho = 1.184;
mu = 0.013;
g = 9.81;

tf = length(x_lead);
u(1:tf-1,1) = design_vector(1:tf-1,1);
u(tf,1) = 0;
s(1,1) = s0;
s(2:tf,1) = design_vector(tf:2*tf-2,1);
v(1,1) = v0;
v(2:tf,1) = design_vector(2*tf-1:3*tf-3,1);


ceq = zeros(2*(tf-1),1);
a = zeros(tf-1,1);
for i = 1:tf-1
    ceq(i,1) = s(i+1,1) - s(i,1) - Ts*v(i,1);
end
for i = 1:tf-1
    a(i,1) = (-(0.5/M)*rho*Cd*Av*v(i,1)^2 - mu*g + u(i));
    ceq(tf-1+i,1) = v(i+1,1) - v(i,1) - Ts*a(i,1);
end

c = zeros(tf-1,1);
for i = 1:tf-2
    c(i,1) = s(i+1,1) - x_lead(i+1,1) + dmin + alpha*v(i+1,1);
end
c(tf-1,1) = df - s(tconstraint);


end