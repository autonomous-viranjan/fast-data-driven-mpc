%% System
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
% constraints
xmax = [5000;30]; xmin = [0;0]; umax = 3; umin = -2;
Xmax = repmat(xmax,2*T,1); Xmin = repmat(xmin,2*T,1);
Umax = umax*ones(2*T,1); Umin = umin*ones(2*T,1);
% Cost
Q = eye(2); R = 1;
Q_big = kron(eye(T),Q); R_big = eye(T);

%% System with reference concatenation
A_bigger = [A_big zeros(2*T,2*T);zeros(2*T,2*T) eye(2*T)]; 
B_bigger = [B_big zeros(2*T,T);zeros(2*T,T) zeros(2*T,T)];

Q_bigger = [Q_big -Q_big;-Q_big Q_big];
R_bigger = [R_big zeros(T);zeros(T) zeros(T)];
%% Fast MPC
SYS.A = A_big;
SYS.B = B_big;
SYS.Q = Q_big;
SYS.R = R_big;
SYS.xmax = Xmax;
SYS.xmin = Xmin;
SYS.umax = Umax;
SYS.umin = Umin;
SYS.n = T*4;
SYS.m = T*2;

params.T = 1; 
params.Qf = Q_bigger;         % final state cost
params.kappa = 0.01;   % barrier parameter
params.niters = 5;     % number of newton steps
params.quiet = false;

X0 = reshape([so(1:T)';vo(1:T)'],[2*T,1]);
U0 = uo(1:T);
x0 = reshape([so(1:T)';vo(1:T)'],[2*T,1]);
%%
[X,U,telapsed] = fmpc_step(SYS,params,X0,U0,x0);
%% Plot Sol
upper = [so(1:T)';vo(1:T)'];
fmpcsol = reshape(X,[2,20]);
figure(1)
plot(upper(1,:),'--')
hold on
plot(fmpcsol(1,:))
figure(2)
plot(upper(2,:),'--')
hold on
plot(fmpcsol(2,:))
figure(3)
plot(uo(1:T),'--')
hold on
plot(U)