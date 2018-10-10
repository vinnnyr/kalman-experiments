%% truckModel.m
x0 = 5; 
dt = 0.01;%s
times = 0:dt:100;

Xk = x0;
Yk = x0;
Fk = 1;
Hk = eye(length(x0));
Qk = 1e-5;
Rk = .01;
Xhat = 5;
Ek = 0;
Kk = 1;
noiseVecProcess = normrnd(0,Qk,1,length(times));
noiseVecSensor = normrnd(0,Rk,1,length(times));

%% Main Loop:
for i = 1:length(times)
    Wk1 = noiseVecProcess(i);
    Zk = noiseVecSensor(i);
    
    Xk(:,end+1) = Fk * Xk(:,1) + Wk1;%Ground Truth
    Yk(:,end+1) = Hk * Xk(:,end) + Zk; %Simulated Observations

    % Prediction
    Xhat(:,end+1) = Fk * Xhat(:,end);
    Ek = Fk * Ek * (Fk.') + Qk;

    % Update
    Ik = Yk(:,end) - Hk*Xhat(:,end); %Innovation
    Sk = Hk*Ek*(Hk.') + Rk; %Innovation Covar
    Kk(:,end+1) = Ek*(Hk.')*(inv(Sk)); %Kalman Gain
    
    Xhat(:,end) = Xhat(:,end) + Kk(:,end)*Ik; %Update estimated X
    Ek = ((eye(length(Xhat(:,end)))) - Kk(:,end)*Hk)*Ek; 
end
%% Plotting
colormap(gray);
 plot(times,Yk(1:end-1),'x'); hold on;
plot(times,Xhat(1:end-1),'-.','LineWidth',2); plot(times,Xk(1:end-1),'-','LineWidth',2)
grid on; grid minor; hold off; legend('Observations','Filter','Truth');
xlabel('Time (s)','FontSize',14);ylabel('Voltage (V)','FontSize',14);

