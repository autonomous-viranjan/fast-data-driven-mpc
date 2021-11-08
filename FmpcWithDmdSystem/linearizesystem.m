function sysd = linearizesystem(Cd,rho,Av,M,vo)
% Linearized System with x~ and u~
A  = cell(1,20);
for i = 1:20
    A{i} = [0 1;0 -(Cd*rho*Av/M)*vo(i)];
end
B = [0;1];
C = eye(2);
sysc = cell(20,1);
for i = 1:20
    sysc{i} = ss(A{i},B,C,0);
end
% Discretized System
sysd = cell(20,1);
for i = 1:20
    sysd{i} = c2d(sysc{i},1);
end    
end