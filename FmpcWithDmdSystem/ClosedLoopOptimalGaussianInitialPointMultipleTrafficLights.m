%{
Alejandro Fernandez Canosa
Research 2017-2018: Illinois Institute of Technology
MainProgramMultipleTrafficLights.m calls the RandomMethodFunction.m
iteratively to compute the position, speed, and control inputs of the host
car.
%}

clear all
clc

ClosedLoopGaussianMultipleTrafficLights;
seed = 1;
rng(seed);
dMin  = 5;    % dmin: lead-host cars
alpha = 0.5;  % dynamic gap

uMin  = -2;     
uMax  = 3;
vMin  = 0;
vMax  = 20;

InitialVelocity = 10;    % Initial velocity of the host car
InitialPosition = 5;    % Initial position of the host car
%% Lead Car data
goal0 = 200;             % Final goal to cross the red lights 
                         % Separated by 200 m
goalGap = 5;             % Practical gap

Ts = 1.0;                % Sample time
tfsecF = 400;            % Simulation time until red light
tfsec  = 20;
NumberTrafficLights = tfsecF/tfsec;
time = (0:Ts:tfsecF)';
time1 = (0:Ts:tfsecF + 100)';
tf = (tfsec/Ts)+1;

goal = (goal0 + goalGap:goal0:goal0*NumberTrafficLights + goalGap)';
RedLights = goal - goalGap;


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
%%
lb(1:tf-1,1)            = uMin*ones(tf-1,1);
lb(tf:2*tf-2,1)         = zeros(tf-1,1);
lb(2*tf-1:3*tf-3,1)     = vMin*ones(tf-1,1);
ub(1:tf-1,1)            = uMax*ones(tf-1,1);
ub(tf:2*tf-2,1)         = 8000*ones(tf-1,1);
ub(2*tf-1:3*tf-3,1)     = vMax*ones(tf-1,1);

% Optimization
options = optimoptions('fmincon');
options = optimoptions(options,'OptimalityTolerance', 1e-6);
options = optimoptions(options,'FunctionTolerance', 1e-6);
options = optimoptions(options,'StepTolerance', 1e-12);  
options = optimoptions(options,'MaxFunctionEvaluations', 1e5);
% % options = optimoptions(options,'FiniteDifferenceStepSize', 1e-02);
options = optimoptions(options,'Algorithm', 'interior-point');

sHostTotal = zeros(tf,1);
vHostTotal = zeros(tf,1);
uHostTotal = zeros(tf-1,1);

tic;
for i = 1:NumberTrafficLights-10
    
    fprintf('Traffic Light = %i', i);
    
    sHost(1,1) = InitialPosition;
    vHost(1,1) = InitialVelocity;
    
    for k = 1:tf-1
        
        initial_point(1:tf-1,1)        = uHostGaussianIn(1 + k - 1 + (i-1)*(tf-1):(i-1)*(tf-1) + tf - 1 + k - 1);
        initial_point(tf:2*tf-2,1)     = sHostGaussianIn(2 + k - 1 + (i-1)*(tf-1):(i-1)*(tf-1) + tf + k - 1);
        initial_point(2*tf-1:3*tf-3,1) = vHostGaussianIn(2 + k - 1 + (i-1)*(tf-1):(i-1)*(tf-1) + tf + k - 1);
            
        COST = @(design_vector)CostFunction(design_vector,...
            InitialPosition,InitialVelocity,goal(i,1),...
            tf,sLead(k + (i-1)*(tf-1):(i-1)*(tf-1) + tf + k - 1,1));        
        
        CONST = @(design_vector)Constraint(design_vector,...
            sLead(k + (i-1)*(tf-1):(i-1)*(tf-1) + tf + k - 1,1),...
            InitialPosition,InitialVelocity,goal(i,1),Ts,dMin,alpha,tf-(k-1));
        
        [design_sol,fval] = ...
            fmincon(COST,initial_point,[],[],[],[],lb,ub,CONST,options);
        
        
        
        uHost(k,1) = design_sol(1,1);
        InitialPosition = design_sol(tf,1); 
        sHost(k+1,1) = InitialPosition;
        InitialVelocity = design_sol(2*tf-1,1);
        vHost(k+1,1) = InitialVelocity;
        
    end
       
    sHostTotal(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k + 1,1) = sHost;
    vHostTotal(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k + 1,1) = vHost;
    uHostTotal(k + 1 - (tf - 1) + (i-1)*(tf-1):(i-1)*(tf-1) + k,1) = uHost;
    
end
CPUTime = toc;

CPUTimeIteration = CPUTime/(NumberTrafficLights*(tf-1));
MPGOptimal = FuelEfficiencyMPGFordFocus(sHostTotal, vHostTotal, uHostTotal, Ts);


