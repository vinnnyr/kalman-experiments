%% truckModel.m
x0 = [0; 0];
dt = 0.01;%s
times = 0:dt:100;

Xk = x0;
Yk = [x0(1);0];
Fk = [1 dt;0 1];
Hk = [1 0;0 0];
Qk = 1e-5;
Rk = .01;
Xhat = [0;0];
Ek = [0 0 ;0 0];
Kk = [1;1];
noiseVecProcess = normrnd(0,Qk,2,length(times));
noiseVecSensor = normrnd(0,Rk,1,length(times));

%% Main Loop:
for i = 1:length(times)
    Wk1 = noiseVecProcess(i);
    Zk = [noiseVecSensor(i);0];
    
    Xk(:,end+1) = Fk * Xk(:,end) + Wk1;%Ground Truth
    Yk(:,end+1) = Hk * Xk(:,end) + Zk; %Simulated Observations

    % Prediction
    Xhat(:,end+1) = Fk * Xhat(:,end);
    Ek = Fk * Ek * (Fk.') + Qk;

    % Update
    Ik = Yk(:,end) - Hk*Xhat(:,end); %Innovation
    Sk = Hk*Ek*(Hk.') + Sk; %Innovation Covar
    Kk = Ek*(Hk.')*(inv(Rk)); %Kalman Gain
    
    Xhat(:,end) = Xhat(:,end) + Kk*Ik; %Update estimated X
    Ek = ((eye(length(Xhat(:,end)))) - Kk*Hk)*Ek; 
end
%% Plotting

figure();

plot(times,Yk(1,1:end-1),'x'); hold on;
plot(times,Xhat(1,1:end-1),'-.','LineWidth',2); plot(times,Xk(1,1:end-1),'-','LineWidth',2)
grid on; grid minor; hold off; 
colormap winter
legend('Observations','Filter','Truth');
xlabel('Time (s)','FontSize',14);ylabel('Position (m)','FontSize',14);


