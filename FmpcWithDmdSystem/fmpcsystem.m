function [SYS,params] = fmpcsystem(sLead0,s0,v0,goal,k)
%function [SYS,params] = fmpcsystem(sLead0,s0,v0,Uhigh,Xhigh,goal,k)
% System
T = 20;
A = cell(T);
B = cell(T);
sysA = [1.00000000000000,0.999999999999999;-7.81383292013652e-07,0.983470415896269];
sysB = [5.32907051820075e-14;0.995353808972818];
for r = 1:T
    for c = 1:T
        if c == r
            A{r,c} = sysA;
            B{r,c} = sysB;
        else
            A{r,c} = zeros(2);
            B{r,c} = zeros(2,1);
        end
    end
end
A_big = cell2mat(A);
B_big = cell2mat(B);
% Constraints
smin = s0'-2;
smin(T-k+1) = goal;
vmin = v0'-3;
smax = s0'+2;
smax(T-k+1) = goal+5;
vmax = v0'+5;

% % concatenated constraints
% Xmin = [reshape([smin;vmin],[2*T,1]);(Xhigh-10e-10)];
% Xmax = [reshape([smax;vmax],[2*T,1]);(Xhigh+10e-10)];
% Umin = [-2*ones(T,1);(Uhigh-10e-10)]; 
% Umax = [3*ones(T,1);(Uhigh+10e-10)];

% concatenated constraints
Xmin = reshape([smin;vmin],[2*T,1]);
Xmax = reshape([smax;vmax],[2*T,1]);
Umin = -2*ones(T,1); 
Umax = 3*ones(T,1);

% Cost
Q = [0 0;0 0]; R = 0.5;
Q_big = kron(eye(T),Q); 
R_big = kron(eye(T),R);

% % System with reference concatenation
% A_bigger = [A_big zeros(2*T,2*T);zeros(2*T,2*T) eye(2*T)]; 
% B_bigger = [B_big zeros(2*T,T);zeros(2*T,T) zeros(2*T,T)];
% Q_bigger = [Q_big zeros(2*T,2*T);zeros(2*T,2*T) zeros(2*T,2*T)];
% R_bigger = [R_big R_big;R_big R_big];
% %R_bigger = [R_big zeros(T);zeros(T) zeros(T)];

% Fast MPC SYS
SYS.A = A_big;
SYS.B = B_big;
SYS.Q = Q_big;
SYS.R = R_big;
SYS.xmax = Xmax;
SYS.xmin = Xmin;
SYS.umax = Umax;
SYS.umin = Umin;
SYS.n = 2*T;
SYS.m = T;

% parameters
params.T = 1; 
params.Qf = Q_big;         % final state cost
params.kappa = 0.01;   % barrier parameter
params.niters = 5;     % number of newton steps
params.quiet = false;

end