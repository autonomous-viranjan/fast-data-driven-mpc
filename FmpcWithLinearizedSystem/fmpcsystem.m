function [SYS,params] = fmpcsystem(sysd,sLead0,s0,Xhigh,Uhigh)
% System
T = 20;
A = cell(T);
B = cell(T);
for r = 1:T
    for c = 1:T
        if c == r
            A{r,c} = sysd{r}.A;
            B{r,c} = sysd{r}.B;
        else
            A{r,c} = zeros(2);
            B{r,c} = zeros(2,1);
        end
    end
end
A_big = cell2mat(A);
B_big = cell2mat(B);

% Constraints on s~ and v~
%smin = -0.01*(sLead0'-s0');
smin = zeros(1,T);
vmin = -10*ones(1,T);
smax = 0.8*(sLead0'-s0');
vmax = 10*ones(1,T);

% concatenated constraints
Xmin = [reshape([smin;vmin],[2*T,1]);(Xhigh-10e-10)];
Xmax = [reshape([smax;vmax],[2*T,1]);(Xhigh+10e-10)];
Umin = [-0.01*ones(T,1);(Uhigh-10e-10)]; 
Umax = [0.01*ones(T,1);(Uhigh+10e-10)];

% Cost
Q = [0 0;0 0]; R = 1;
Q_big = kron(eye(T),Q); 
R_big = kron(eye(T),R);

% System with reference concatenation
A_bigger = [A_big zeros(2*T,2*T);zeros(2*T,2*T) eye(2*T)]; 
B_bigger = [B_big zeros(2*T,T);zeros(2*T,T) zeros(2*T,T)];
Q_bigger = [Q_big zeros(2*T,2*T);zeros(2*T,2*T) zeros(2*T,2*T)];
R_bigger = [R_big R_big;R_big R_big];
%R_bigger = [R_big zeros(T);zeros(T) zeros(T)];

% Fast MPC SYS
SYS.A = A_bigger;
SYS.B = B_bigger;
SYS.Q = Q_bigger;
SYS.R = R_bigger;
SYS.xmax = Xmax;
SYS.xmin = Xmin;
SYS.umax = Umax;
SYS.umin = Umin;
SYS.n = 4*T;
SYS.m = 2*T;

% parameters
params.T = 1; 
params.Qf = Q_bigger;         % final state cost
params.kappa = 0.01;   % barrier parameter
params.niters = 5;     % number of newton steps
params.quiet = false;

end