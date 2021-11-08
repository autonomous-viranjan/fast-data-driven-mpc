% Author : Viranjan Bhattacharyya
% Year : 2020
clear all
clc

%ClosedLoopGaussianMultipleTrafficLights;
seed = 1;
NumberBranches = 1e3;
NumberTrials = 200;

uMin  = -2;     
uMax  = 3;
vMin  = 0;
vMax  = 20;

InitialVelocity = 5;    % Initial velocity of the host car
InitialPosition = 5;    % Initial position of the host car

goal0 = 200;             % Final goal to cross the red lights 
                         % Separated by 200 m
goalGap = 5;             % Practical gap

Ts = 1.0;                % Sample time
tfsecF = 400;            % Simulation time until red light
tfsec  = 20;
NumberTrafficLights = tfsecF/tfsec;
time = (0:Ts:tfsecF)';
time1 = (0:Ts:tfsecF + 100)';
tf = (tfsec/Ts) + 1;
T = 20;

%dMin = 6;    % constant sigma
%dMin over the horizon -- time-varying sigma
dMin  = zeros(T,1);    
alpha = 0.3;  % dynamic gap

L = 5;
sigma = 1;
beta = 0.01;

for i=1:T
    %dMin(i) = L/2 + (1.01^(i-1))*sigma*sqrt(log(L^2/(2*pi*(sigma^2)*(beta^2))));
    dMin(i) = L/2 + (1.01^(i-1))*sigma*sqrt(log(L^2/(2*pi*(((1.01^(i-1))*sigma)^2)*(beta^2))));
end
%
goal = (goal0 + goalGap:goal0:goal0*NumberTrafficLights + goalGap)';
RedLights = goal - goalGap;

rng(seed);
% Driving cycle of the lead car
% Initial conditions of the lead car
v0Lead = 10; % m/s
s0Lead = 40; % m
vLead = v0Lead - 3.2*sin(0.3*time1) + 0.2*randn(length(time1),1);
% vLead = v0Lead*ones(tfsecF+1,1);

sLead = zeros(length(time1),1);
sLead(1,1) = s0Lead;
for i = 2:length(time1)
    sLead(i,1) = sLead(i-1,1) + Ts*vLead(i-1,1);
end

M   = 1200;
Av  = 2.5;
Cd  = 0.32;
rho = 1.184;
mu  = 0.013;
g   = 9.81;

uLead = zeros(length(time1)-1,1);
for i = 1:length(time1)-1
    uLead(i,1) = ((vLead(i+1,1) - vLead(i,1))/Ts) + (1./(2.*M))*rho*Cd*Av*vLead(i,1)^2 + mu*g;
end
MPGLead = FuelEfficiencyMPGFordFocus(sLead, vLead, uLead, Ts);

sEgoFmpc = zeros(tf,1);
vEgoFmpc = zeros(tf,1);
uEgoFmpc = zeros(tf-1,1);
uHost = zeros(T,1);
x0 = zeros(2*T,1);

tic
for i = 1:NumberTrafficLights
    
    fprintf('Traffic Light = %i', i);
    
    sHost(1,1) = InitialPosition;
    vHost(1,1) = InitialVelocity;
            
    for k = 1:tf-1 
                       
        [sHigh, vHigh, uHigh, MaxMPG, GaussCPU, SuccesfulBranches, Mean1,...
            Mean2, Mean, Sigma, BranchSuccessPos] = ...
            GaussianMethodFunction(time((i-1)*(tf-1) + k:(i-1)*(tf-1) + tf,1),...
            sLead((i-1)*(tf-1) + k:(i-1)*(tf-1) + tf,1),...
            vLead((i-1)*(tf-1) + k:(i-1)*(tf-1) + tf,1),...
            InitialVelocity, InitialPosition,...
            goal(i,1), Ts, seed,...
            NumberBranches, NumberTrials, dMin, alpha,tf-(k-1));
        
        if length(sHigh)<tf
            v=0;
            v(1) = vHigh(end);
            
            if k>2
                for t = 1:k-2
                    v(t+1) = v(t) + Ts*(- (1/(2*M))*Cd*rho*Av*(v(t)^2) - mu*g);
                end
            end
            sHigh = [sHigh;v'+sHigh(end)];
            vHigh = [vHigh;v'];
            uHigh = [uHigh;zeros(k-1,1)];
%             v = 0.6*vHigh(end);
%             v = 0.5*vLead(i*k + 20);
%             v = 9;
%             sHigh = [sHigh;(v*(1:k-1)')+sHigh(end)];
%             vHigh = [vHigh;v*ones(k-1,1)];
%             uHigh = [uHigh;((0.5/M)*Cd*rho*Av*(v)^2 + mu*g)*ones(k-1,1)];
        end
        
        s0 = sHigh(2:end);
        v0 = vHigh(2:end);
        Uhigh = uHigh;
        Xhigh = reshape([s0';v0'],[2*T,1]);
        sLead0 = sLead(2 + k - 1 + (i-1)*(tf-1):(i-1)*(tf-1) + tf + k - 1);
        Xwarm = [zeros(2*T,1);Xhigh];
        Uwarm = [zeros(T,1);Uhigh];
        X0 = [x0;Xhigh];        
            
        sysd = linearizesystem(Cd,rho,Av,M,v0);       
        
        [SYS,params] = fmpcsystem(sysd,sLead0,s0,Xhigh,Uhigh);
        
        [X,U,telapsed] = fmpc_step(SYS,params,Xwarm,Uwarm,X0);
        
        x0 = X(1:2*T);
        
        % Update using vehicle longitudinal model.        
        uHost(k,1) = U(1) +  Uhigh(1);
        InitialPosition = InitialPosition + Ts*InitialVelocity;
        %InitialPosition = X(1) + X0(1); 
        sHost(k+1,1) = InitialPosition;
        InitialVelocity = InitialVelocity + Ts*(uHost(k,1) - (1/(2*M))*Cd*rho*Av*InitialVelocity^2 - mu*g);
        %InitialVelocity = X(2) + X0(2);
        vHost(k+1,1) = InitialVelocity;        
           
    
    end
     
    sEgoFmpc(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k + 1,1) = sHost;
    vEgoFmpc(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k + 1,1) = vHost;
    uEgoFmpc(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k,1) = uHost;
    
end
CPUTime = toc;
%%
CPUTimeIteration = CPUTime/(NumberTrafficLights*(tf-1));
MPGOptimal = FuelEfficiencyMPGFordFocus(sEgoFmpc, vEgoFmpc, uEgoFmpc, Ts);
%% save data
% constant sigma
% save('s_fmpc_cs.mat','sEgoFmpc');
% save('u_fmpc_cs.mat','uEgoFmpc');
% save('v_fmpc_cs.mat','vEgoFmpc');
% time-varying sigma
save('s_fmpc_ts.mat','sEgoFmpc');
save('u_fmpc_ts.mat','uEgoFmpc');
save('v_fmpc_ts.mat','vEgoFmpc');
