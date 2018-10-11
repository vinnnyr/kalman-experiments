%% truckModel.m
x0 = [0; 0; 0; 1;];
dt = 1;%s
times = 0:dt:100;


Fk = [1 0 1 0; 0 1 0 1;0 0 dt 0;0 0 0 dt];
Hk = [1 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0];

Xk = x0;
Yk = Hk * Xk;

Qk = 1e-5;
Rk = .01;

Xhat = x0;
Ek = zeros(4,4);
Sk = zeros(4,4);
Kk = ones(4,1);

noiseVecProcess = normrnd(0,Qk,4,length(times));
noiseVecSensor = normrnd(0,Rk,2,length(times));

%% Main Loop:
for i = 1:length(times)
    Wk1 = noiseVecProcess(:,i);
    Zk = [noiseVecSensor(:,i);0;0];
    
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

plot(Yk(1,:),Yk(2,:),'x'); hold on;
plot(Xhat(1,:),Xhat(2,:),'-.','LineWidth',2); plot(Xk(1,:),Xk(2,:),'-','LineWidth',3)
grid on; grid minor; hold off; 
colormap winter
legend('Observations','Filter','Truth');
xlabel('Position X (m)','FontSize',14);ylabel('Position Y (m)','FontSize',14);

figure();
    
for i = 1:length(times)
    
    
    plot(Yk(1,1:i),Yk(2,1:i),'x'); hold on;
    plot(Xhat(1,1:i),Xhat(2,1:i),'-.','LineWidth',2); %plot(Xk(1,1:i),Xk(2,1:i),'o','LineWidth',3)
    hold off;
    
    grid on; grid minor;
    colormap winter
    legend('Observations','Filter');    
    xlabel('Position X (m)','FontSize',14);ylabel('Position Y (m)','FontSize',14);
    
    drawnow()
    pause(.001)
end


