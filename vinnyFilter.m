function [] = vinnyFilter(observations,A,controlSignal,sensorNoise,times,predictIntial,actualSignal)
%% Vinny Implementation of EKF

predictions = predictIntial; %predictions is a vector where we will store all predictions
predictionError = eye(3); %arbritary
%model model is not a function handle for the moment
R = sensorNoise;
C = eye(3,3);%* (1+eps());

for i = 1:length(times)
   % Predict
   predictions(:,end+1) = A * predictions(:,end) + (controlSignal(:,i));
   predictionErrorNew = A * (C * predictionError) * A.';
   
   % Update
   G = predictionErrorNew * pinv(predictionErrorNew+R);
   predictions(:,end) = predictions(:,end) + (G*(observations(:,i+1) - predictions(:,end)));
   predictionErrorNew = (eye(3) - G)* predictionErrorNew;
   predictionError = predictionErrorNew;
end

errorVec = actualSignal - predictions;

subplot(2,2,1)
plot(times,observations(1,1:end-1),'b.','LineWidth',2);hold on;
plot(times,actualSignal(1,1:end-1),'r-','LineWidth',2); 
plot(times,predictions(1,1:end-1),'k-','LineWidth',2); hold off; 
legend('Observations','Actual Signal','Filtered Data');grid on; grid minor; 
xlabel('Time (s)','FontSize',14);
ylabel('Altitude (m)','FontSize',14); 

subplot(2,2,2)
plot(times,errorVec(1,1:end-1),'b.','LineWidth',2);
legend('Error');grid on; grid minor;
xlabel('Time (s)','FontSize',14);
ylabel('Altitude Error','FontSize',14); 

subplot(2,2,3)
plot(times,observations(2,1:end-1),'b.','LineWidth',2);hold on;
plot(times,actualSignal(2,1:end-1),'r-','LineWidth',2); 
plot(times,predictions(2,1:end-1),'k-','LineWidth',2); hold off; 
legend('Observations','Actual Signal','Filtered Data');grid on; grid minor; 
xlabel('Time (s)','FontSize',14);
ylabel('Velocity (m/s)','FontSize',14); 

subplot(2,2,4)
plot(times,errorVec(2,1:end-1),'b.','LineWidth',2);
legend('Error');grid on; grid minor;
xlabel('Time (s)','FontSize',14);
ylabel('Velocity Error','FontSize',14); 

end