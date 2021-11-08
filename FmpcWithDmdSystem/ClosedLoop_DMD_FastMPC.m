% Author : Viranjan Bhattacharyya
% Aug 2020
clear all
clc

seed = 1;
NumberBranches = 1000;
NumberTrials = 200;

dMin  = 7.5;    % constant sigma
alpha = 0.3;  % dynamic gap

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
%dMin over the horizon -- time-varying sigma
% L = 5;
% sigma = 1;
% beta = 0.01;
% dMin  = zeros(T,1);
% for i=1:T
%     dMin(i) = L/2 + (1.01^(i-1))*sigma*sqrt(log(L^2/(2*pi*(((1.01^(i-1))*sigma)^2)*(beta^2))));
% end
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

sEgoTotaldmd = zeros(tf,1);
vEgoTotaldmd = zeros(tf,1);
uEgoTotaldmd = zeros(tf-1,1);
uHost = zeros(T,1);
%x0 = zeros(2*T,1);
v0(1) = InitialVelocity;
for t = 1:T-1
    v0(t+1) = v0(t) + Ts*(- (1/(2*M))*Cd*rho*Av*(v0(t)^2) - mu*g);
end
s0(1) = InitialPosition;
for t = 1:T-1
    s0(t+1) = s0(t) + v0(t);
end
x0 = reshape([s0;v0],[2*T,1]);
%%
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
            NumberBranches, NumberTrials, dMin, alpha, tf-(k-1));
        
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
%             sHigh = [sHigh;(v*(1:k-1)')+sHigh(end)];
%             vHigh = [vHigh;v*ones(k-1,1)];
%             uHigh = [uHigh;((0.5/M)*Cd*rho*Av*(v^2) + mu*g)*ones(k-1,1)];
        end
        
                
        Xhigh = reshape([sHigh(2:end)';vHigh(2:end)'],[2*T,1]);
        sLead0 = sLead(2 + k - 1 + (i-1)*(tf-1):(i-1)*(tf-1) + tf + k - 1);
        Xwarm = Xhigh;
        Uwarm = uHigh;
        
                
            
        %sysd = linearizesystem(Cd,rho,Av,M,v0);       
        
        [SYS,params] = fmpcsystem(sLead0,sHigh(2:end),vHigh(2:end),goal(i),k);
        
        [X,U,telapsed] = fmpc_step(SYS,params,Xwarm,Uwarm,x0);
        
        
        
        % Update using vehicle longitudinal model.        
        uHost(k,1) = U(1);
        InitialPosition = InitialPosition + Ts*InitialVelocity;
        %InitialPosition = X(1) + X0(1); 
        sHost(k+1,1) = InitialPosition;
        InitialVelocity = InitialVelocity + Ts*(uHost(k,1) - (1/(2*M))*Cd*rho*Av*InitialVelocity^2 - mu*g);
        %InitialVelocity = X(2) + X0(2);
        vHost(k+1,1) = InitialVelocity;
        
        
        x0 = [InitialPosition;InitialVelocity;X(3:2*T)];
           
    
    end
     
    sEgoTotaldmd(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k + 1,1) = sHost;
    vEgoTotaldmd(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k + 1,1) = vHost;
    uEgoTotaldmd(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k,1) = uHost;
    
end
CPUTime = toc;
%%
CPUTimeIteration = CPUTime/(NumberTrafficLights*(tf-1));
MPGOptimal = FuelEfficiencyMPGFordFocus(sEgoTotaldmd, vEgoTotaldmd, uEgoTotaldmd, Ts);
%% save data
% constant sigma
save('s_fmpcdmd_cs.mat','sEgoTotaldmd');
save('u_fmpcdmd_cs.mat','uEgoTotaldmd');
save('v_fmpcdmd_cs.mat','vEgoTotaldmd');
% time-varying sigma
% save('s_fmpcdmd_ts.mat','sEgoTotaldmd');
% save('u_fmpcdmd_ts.mat','uEgoTotaldmd');
% save('v_fmpcdmd_ts.mat','vEgoTotaldmd');
