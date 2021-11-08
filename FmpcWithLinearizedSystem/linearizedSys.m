% Initial Solution
ClosedLoopGaussianMultipleTrafficLights;
so = sHostGaussianIn;
vo = vHostGaussianIn; 
uo = uHostGaussianIn;

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

%% The LTV system
% SYS = cell(20,1);
% for i = 1:20
%     SYS{i}.A = sysd{i}.A;
%     SYS{i}.B = sysd{i}.B;
%     SYS{i}.Q = eye(2); 
%     SYS{i}.R = 1;
%     SYS{i}.xmax = [100;30];   %   state upper limits x_{max}
%     SYS{i}.xmin = [0;0];  %   state lower limits x_{min}
%     SYS{i}.umax = 2.2;  %   input upper limits u_{max}
%     SYS{i}.umin = -2.2; %   input lower limits u_{min}
%     SYS{i}.n = 2;  %   number of states
%     SYS{i}.m = 1;  %   number of inputs
% end
% 
% params.T = 1; 
% params.Qf = eye(2);         % final state cost
% params.kappa = 0.01;   % barrier parameter
% params.niters = 5;     % number of newton steps
% params.quiet = false;
% 
% x0 = [so(1);vo(1)];
%%
%[X,U,telapsed] = fmpc_step(SYS{1},params,[0;0],0,x0);
%% Test with xo = [5;10]
X = cell(1,20);
X{1} = [so(1);vo(1)];
for j = 1:19
    X{j+1} = (sysd{j}.A)*X{j} + (sysd{j}.B)*uo(j);
end
X = cell2mat(X);
pos = X(1,:);
vel = X(2,:);
plot(vel)
hold on
plot(vo(1:20))
figure(2)
plot(pos)
hold on
plot(so(1:20))