%% Tutorial for airplane Kalman Filter
rng(10);
states = [1000;0;-9.81]; %m;m/s
statesWithNoise = states(:,1); %m start with 100 m regardless
dt = 0.01;
times = 1:dt:100;

controlSignal = [];
%controlSignal(1,:) = .99 * times ./ times;
%controlSignal(2,:) =  - 10 * sin(times / (2*pi*9000)); %let's say we are only controlling velocity
%controlSignal(2,1000:3000) = .2; %let's say we are only controlling velocity

controlSignal = zeros(3,length(times));

noisePos = normrnd(0,1,1,length(times));
noiseVel = normrnd(0,1,1,length(times));
noiseAccel = normrnd(0,1e-3,length(times));
noise = [noisePos;noiseVel;noiseAccel];

%turbPos = normrnd(0,.05,1,length(times));
%turbVel = normrnd(-.5,.5,1,length(times));
turbPos = normrnd(0,10,1,length(times));
turbVel = normrnd(0,1,1,length(times));
turbAccel = zeros(1,length(times));

turb = [turbPos;turbVel;turbAccel];

A = eye(3,3);
A(1,2) = dt;
A(2,3) = dt;


for i = 1:length(times)
    states(:,end+1) = A * (states(:,end)) + turb(:,i) + controlSignal(:,i); %Turb is the process nosie
    statesWithNoise(:,end+1) = states(:,end) + noise(i); %noise is the noise noise
end
plotting = false; 
if plotting
    plot(times,statesWithNoise(1,1:length(times)-1),'.');
    hold on; plot(times,statesWithNoise(2,1:length(times)-1),'.');plot(times,states(1,1:end-1),'.');
    plot(times,states(2,1:end-1),'.');
    hold off; legend('Pos (Noisy)','Vel (Noisy)','Pos (Actual)','Vel (Actual)');
end
vinnyFilter(statesWithNoise,A,controlSignal,[10;1;1e-3],times,states(:,1) + 0,states)
