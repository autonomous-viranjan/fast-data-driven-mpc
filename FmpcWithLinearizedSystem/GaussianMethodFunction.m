%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Alejandro Fernandez Canosa
Illinois Institute of Technology
Research 2017-2018
Two-Stages Heuristic Sampling based method
This function will compute the position, speed, and control of the host
car and choose among all the randomly selected branches, the one with the best
fuel economy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

function [Space, Velocity, Control, MaxMPG, CPUTime, SuccessfulBranches, Mean1, Mean2, Mean, Sigma, BranchSuccessPos] = ...
    GaussianMethodFunction(time, xLead, vLead, v0, s0, goal, Ts, ...
    seed, NumberBranches, NumberTrials, dMin, alpha, tc)

uMin  = -2;     
uMax  = 3;
vMin  = 0;
vMax  = 20;

% Parameters
M   = 1200;
Av  = 2.5;
Cd  = 0.32;
rho = 1.184;
mu  = 0.013;
g   = 9.81;

tf = length(time);
tfsec = (tf-1)*Ts;

BranchPos = zeros(tf,NumberBranches);
BranchSpeed = zeros(tf,NumberBranches);
BranchControl = zeros(tf-1,NumberBranches);

n = 0; 
rng(seed) % Seed of the random number generator

Mean  = zeros(tf-1,1);
Mean1 = zeros(tf-1,1);
Mean2 = zeros(tf-1,1);
vTarget = zeros(tf-1,1);

tic;
for i = 1:NumberBranches   
    
    pos = s0;
    speed = v0;
    posPrev = pos;
    speedPrev = speed;
    
    BranchPos(1,i) = s0;
    BranchSpeed(1,i) = v0;   
    
    for k = 2:tf   
        
        vTarget(k-1,i) = (goal - pos)/(tfsec - Ts*(k-2));
        if goal < pos
            %vTarget(k-1,i) = vTarget(k-2,1);
            vTarget(k-1,i) = 0;
        end
            
        Mean1(k-1,i) = ((vTarget(k-1,1) - speed)/Ts) + (1./(2.*M))*Cd*rho*Av*speed^2 + mu*g;
        if Mean1(k-1,i) > 0
            Mean1(k-1,i) = min(Mean1(k-1,i), uMax);
        else
            Mean1(k-1,i) = max(Mean1(k-1,i), uMin);
        end
        
        Mean2(k-1,i) = ((vLead(k-1,1) - speed)/Ts) + (1./(2.*M))*Cd*rho*Av*speed^2 + mu*g;
        if Mean2(k-1,i) > 0
            Mean2(k-1,i) = min(Mean2(k-1,i), uMax);
        else
            Mean2(k-1,i) = max(Mean2(k-1,i), uMin);
        end
        
        p = 1;
        %if xLead(k-1,1) - pos >= p*(dMin + alpha*speed)
        if xLead(k-1,1) - pos >= p*(dMin(k-1) + alpha*speed)
            Mean(k-1,i) = Mean1(k-1,i);
        else
            Mean(k-1,i) = Mean2(k-1,i);
        end
        
        d = xLead(k-1,1) - pos;
        %dConst = dMin; % constant sigma
        %dConst = dMin(k-1) + alpha*speed;
        dConst = dMin(k-1); % time-varying sigma
        
        beta1 = 0.6;
        beta2 = 1.4;
        Sigma(k-1,i) = beta1*abs(dConst/(d - beta2*dConst));
                            
        for j = 1:NumberTrials
            
            control = Mean(k-1,i) + Sigma(k-1,i)*randn;
            
            pos = posPrev + Ts*speedPrev;
            a = control - (1./(2.*M))*Cd*rho*Av*speedPrev^2 - mu*g;
            speed = speedPrev + Ts*a;
            
            %if (xLead(k,1) - pos >= dMin + alpha*speed && ...
            if (xLead(k,1) - pos >= dMin(k-1) + alpha*speed && ...                       
                speed >= vMin && speed <= vMax && ...
                control >= uMin && control <= uMax)
            
                BranchPos(k,i) = pos;
                BranchSpeed(k,i) = speed;
                BranchControl(k-1,i) = control;
                posPrev = pos;
                speedPrev = speed;
                break;
                
            end          
        end
    end
    
    if BranchPos(tc,i) >= goal
        n = n + 1;
        BranchSuccessPos(:,n)     = BranchPos(:,i);
        BranchSuccessSpeed(:,n)   = BranchSpeed(:,i);
        BranchSuccessControl(:,n) = BranchControl(:,i);
    end
    
end

n1 = 0;
for i = 1:n
    n1 = n1 + 1;
    for k = 2:tf
        if BranchSuccessPos(k,i) ~= 0  
            BranchSuccessPos(k,n1) = BranchSuccessPos(k,i);
            BranchSuccessSpeed(k,n1) = BranchSuccessSpeed(k,i);
            BranchSuccessControl(k-1,n1) = BranchSuccessControl(k-1,i);
        else
            n1 = n1 - 1;
            break;
        end
    end
end

CPUTime = toc;

MPG = zeros(1,n1);
for i = 1:n1
    MPG(i) = FuelEfficiencyMPGFordFocus(BranchSuccessPos(:,i),BranchSuccessSpeed(:,i),BranchSuccessControl(:,i),Ts);
end
% Calculate the max MPG branch
[MaxMPG,ind] = max(MPG);

Space    = BranchSuccessPos(:,ind);
Velocity = BranchSuccessSpeed(:,ind);
Control  = BranchSuccessControl(:,ind);
SuccessfulBranches = 100*(n1/NumberBranches);
    


end


