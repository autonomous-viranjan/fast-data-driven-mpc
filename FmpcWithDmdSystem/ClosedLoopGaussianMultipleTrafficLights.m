%{
Alejandro Fernandez Canosa
Research 2017-2018: Illinois Institute of Technology
MainProgramMultipleTrafficLights.m calls the RandomMethodFunction.m
iteratively to compute the position, speed, and control inputs of the host
car.
%}

clear, clc

seed = 1;
NumberBranches = 1e3;
NumberTrials = 200;

dMin  = 5;    % dmin: lead-host cars
alpha = 0.5;  % dynamic gap

InitialVelocity = 10;    % Initial velocity of the host car
InitialPosition = 5;    % Initial position of the host car

goal0 = 200;             % Final goal to cross the red lights 
                         % Separated by 200 m
goalGap = 5;             % Practical gap

Ts = 1.0;                % Sample time
tfsecF = 420;            % Simulation time until red light
tfsec  = 20;
NumberTrafficLights = tfsecF/tfsec;
time = (0:Ts:tfsecF)';
tf = (tfsec/Ts) + 1;

goal = (goal0 + goalGap:goal0:goal0*NumberTrafficLights + goalGap)';
RedLights = goal - goalGap;

rng(seed);
% Driving cycle of the lead car
% Initial conditions of the lead car
v0Lead = 10; % m/s
s0Lead = 40; % m
vLead = v0Lead - 3.2*sin(0.3*time) + 0.2*randn(length(time),1);
% vLead = v0Lead*ones(tfsecF+1,1);

sLead = zeros(length(time),1);
sLead(1,1) = s0Lead;
for i = 2:length(time)
    sLead(i,1) = sLead(i-1,1) + Ts*vLead(i-1,1);
end

M   = 1200;
Av  = 2.5;
Cd  = 0.32;
rho = 1.184;
mu  = 0.013;
g   = 9.81;

uLead = zeros(length(time)-1,1);
for i = 1:length(time)-1
    uLead(i,1) = ((vLead(i+1,1) - vLead(i,1))/Ts) + (1./(2.*M))*rho*Cd*Av*vLead(i,1)^2 + mu*g;
end
MPGLead = FuelEfficiencyMPG(sLead, vLead, uLead, Ts);

CPUTotal = 0;
PercentageSuccessfulBranches = 0;
MPG = zeros(NumberTrafficLights, 1);

sHostTotal = zeros(tf,1); sHostTotal(1,1) = InitialPosition;
vHostTotal = zeros(tf,1); vHostTotal(1,1) = InitialVelocity;
uHostTotal = zeros(tf-1,1);

for i = 1:NumberTrafficLights
    
    for k = 1:tf-1
    
        [sHost, vHost, uHost, MaxMPG, CPU, SuccesfulBranches, Mean1,...
            Mean2, Mean, Sigma, BranchSuccessPos] = ...
            GaussianMethodFunction(time((i-1)*(tf-1) + k:(i-1)*(tf-1) + tf,1),...
            sLead((i-1)*(tf-1) + k:(i-1)*(tf-1) + tf,1),...
            vLead((i-1)*(tf-1) + k:(i-1)*(tf-1) + tf,1),...
            InitialVelocity, InitialPosition,...
            goal(i,1), Ts, seed,...
            NumberBranches, NumberTrials, dMin, alpha,tf-(k-1));
    
        InitialVelocity = vHost(2,1);
        InitialPosition = sHost(2,1);
    
        sHostTotal((i-1)*(tf-1) + k+1,1) = sHost(2,1);
        vHostTotal((i-1)*(tf-1) + k+1,1) = vHost(2,1);
        uHostTotal((i-1)*(tf-1) + k,1)   = uHost(1,1);
    
        CPUTotal = CPUTotal + CPU;
        PercentageSuccessfulBranches = PercentageSuccessfulBranches + SuccesfulBranches;
    
    end
    MPG(i,1) = FuelEfficiencyMPGFordFocus(sHostTotal, vHostTotal, uHostTotal, Ts);
end

TotalMPG = FuelEfficiencyMPGFordFocus(sHostTotal, vHostTotal, uHostTotal, Ts);
CPUIteration = CPUTotal/(NumberTrafficLights*(tf-1));
PercentageSuccessfulBranches = PercentageSuccessfulBranches/(NumberTrafficLights*tf);

sConst = dMin + alpha*vHostTotal;
sActual = sLead - sHostTotal;

vHostTotalCL = vHostTotal;

uHostGaussianIn = uHostTotal;
sHostGaussianIn = sHostTotal;
vHostGaussianIn = vHostTotal;


