%% Training Data
load('s_data.mat');
load('v_data.mat');
load('u_data.mat');
%%
plot(sHostTotal)
figure(2)
plot(vHostTotal)
figure(3)
plot(uHostTotal)
%% Training
X = [sHostTotal(1:end-1)';vHostTotal(1:end-1)'];
X_dash = [sHostTotal(2:end)';vHostTotal(2:end)'];
U_train = uHostTotal';
Omega = [X;U_train];
G = X_dash*pinv(Omega);
A = G(1:2,1:2); B = G(1:2,3);
%% Predict
u = uHostTotal(20:29)';
Xp = cell(1,length(u)); 
Xp{1}=[sHostTotal(20);vHostTotal(20)];

for j=1:length(u)
    Xp{j+1}=A*Xp{j}+B*u(j);
end
p = cell2mat(Xp);
sp = p(1,:);
vp = p(2,:);
%% Plot
figure(1)
plot(vp)
hold on
plot(vHostTotal(20:30)')
legend('Predicted','Actual')
figure(2)
plot(sp)
hold on
plot(sHostTotal(20:30)')