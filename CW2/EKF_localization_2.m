% 23TTP409 coursework 2: EKF localisation 
% Gabriel Wendel

clear all
close all
clc

%--------------------------------------------------------------------------
% simulation settings 

timeStep = 150; % total simulation steps
Ts=0.1; % sampling interval 


sig_bearing = 0.045; % standard deviation of bearing noise (rad) orignially 0.045
sig_odometry = 0.7; % standard deviation of odometry noise (m/s) originally 0.7
%--------------------------------------------------------------------------

% Landmark locations 

nLandmark = 3; % three landmarks are considered 

% define the default landmark map based on random points 
% inital map area: -50m to 50m on both x and y directions
landmarkMap = 100*rand(2,nLandmark)-50;

% re-define three landmarks in designated positions
landmarkMap(:,1) = [20*rand-60; 20*rand+20];
landmarkMap(:,2) = [20*rand+10; 20*rand+15];
landmarkMap(:,3) = [20*rand+20; 20*rand-25];

% plot the landmarks
figure(1)
hold on
axis equal 
xlabel('x position (m)')
ylabel('y position (m)')

for kk = 1:nLandmark 
    plot(landmarkMap(1,kk),landmarkMap(2,kk),'d','LineWidth',2);
    str = sprintf('The position of landmark No.%d is (%0.5g, %0.5g)', kk, landmarkMap(1,kk), landmarkMap(2,kk));
    disp(str);
end
%--------------------------------------------------------------------------

% Generate vehicle trajecotry 

% define the array that contains the vehicle trajectory

% generate vehicle trajecotry using the unicycle vehicle model 
xpos = -30; % x position 
ypos = -5; % y position
xvel = 0;
yvel = 5;
vel = 5; % vehicle speed
psi = atan2(yvel,xvel); % vehicle heading

xTrue = [];
for kk = 1:timeStep    
    % define heading angle at different time steps
    if kk<=20
        omg = 0; 
    elseif kk<=80
        omg = -pi/12;       
    elseif kk<=100
        omg = 0;
        vel = vel + 0.2;
    elseif kk<=110
        omg = -pi/2;
    else 
        omg = 0;
    end
    
    xpos = xpos + vel*cos(psi)*Ts;
    ypos = ypos + vel*sin(psi)*Ts;
    psi = psi + omg*Ts;
    
    newState = [xpos ypos vel*cos(psi) vel*sin(psi) vel omg]';
    
    xTrue = [xTrue newState]; % vehicle state trajectory
end

plot(xTrue(1,:),xTrue(2,:),'LineWidth',2);
legend('Landmark 1','Landmark 2','Landmark 3','Vehcile position', 'Location','south');
%--------------------------------------------------------------------------

% Generate sensor readings 

% define the 2-D array that stores the bearing readings w.r.t all landmarks
% and the velocity readings by the odometry
% at each time step zObv = [bearing1; bearing2; bearing3];
 
zObv = zeros(nLandmark,size(xTrue,2)); % bearing 

% generate sensor readings for each landmark
for kk = 1:nLandmark
    xLandmark = landmarkMap(1,kk); % landmark x position
    yLandmark = landmarkMap(2,kk); % landmark y position
    xDist = xLandmark-xTrue(1,:);
    yDist = yLandmark-xTrue(2,:);
    zObv(kk,:) = atan2(yDist,xDist) + sig_bearing*randn(1,timeStep);
end

%==========================================================================
% Evaluation
%==========================================================================

% Note: to make sure the evaluation process can be carried out smoothly, 
% please put landmark_localization_EKF_eval.p in your current folder 

% uncomment the following three lines of code to evaluate of your code using another dataset

% clear xTrue landmarkMap zObv
% close all
% 
% 
% [xTrue, landmarkMap, zObv] = landmark_localization_EKF_eval(1234); % input the last four digits of your student number

%=========== Write your code here to process the sensor readings ==========

% Initialize EKF parameters
xInit = [-30 -5 atan2(5,0)]'; % initial state
xhat = xInit; % Initial state estimate (position x, position y and heading angle)
P = eye(3); % initial covariance matrix

omega = 0; % yaw rate

% Process noise covariance (Q)
Q = diag([sig_odometry^2, sig_odometry^2, sig_odometry^2]); % process noise covariance

% Measurement noise covariance (R)
R = sig_bearing^2 * eye(nLandmark)*50;

% Initialize output variables
xhatOut = zeros(3, timeStep);

% EKF Algorithm
for k = 1:timeStep
    % Prediction Step
    % State prediction based on bicycle model 
    xhat(1) = xhat(1) + vel*cos(xhat(3))* Ts;
    xhat(2) = xhat(2) + vel*sin(xhat(3))*Ts;
    xhat(3) = xhat(3) + omega * Ts;
    
    
    % Update covariance prediction based on the motion model
    F = [1, 0, -vel*sin(xhat(3))*Ts;
         0, 1, vel*cos(xhat(3))*Ts;
         0, 0, 1]; % Jacobian of the motion model
    P_minus = F * P * F' + Q; % Covariance prediction
    
    % Update Step
    for kk = 1:nLandmark
        % Measurement model (h(x))
        dx = landmarkMap(1,kk) - xhat(1);
        dy = landmarkMap(2,kk) - xhat(2);
        range = sqrt(dx^2 + dy^2);
        H = [dy/range^2, -dx/range^2, 0]; % Jacobian of the measurement model per landmark
        %H = [dx/sqrt(dx^2 + dy^2) dy/sqrt(dx^2 + dy^2) 0 0;
        %    -dy/(dx^2 + dy^2), dx/(dx^2 + dy^2), 0, 0];
        
        % Predicted measurement
        zhat = atan2(dy, dx);
        
        % Measurement residual
        z = zObv(kk, k);
        z_res = z - zhat;
        
        % Ensure the residual is within -pi to pi
        if z_res > pi
            z_res = z_res - 2*pi;
        elseif z_res < -pi
            z_res = z_res + 2*pi;
        end
        
        % Kalman Gain
        K = P_minus * H' / (H * P_minus * H' + R(kk,kk));
        
        % Update state estimate and covariance
        xhat = xhat + K * z_res;
        P = (eye(3) - K * H) * P_minus;
    end
    
    % Store the estimated state at each time step
    xhatOut(:, k) = xhat;
end

% Plot estimated trajectory
figure(1)
plot(xhatOut(1,:), xhatOut(2,:), '--', 'LineWidth', 2);
legend('Landmark 1', 'Landmark 2', 'Landmark 3', 'Vehcile position', 'Estimated trajectory', 'Location', 'south');
xlabel('x position (m)');
ylabel('y position (m)');
axis equal;

xhatvel = vel*cos(xhatOut(3,:));
yhatvel = vel*sin(xhatOut(3,:));

%% postprocess of the results 
% uncomment to use the code to plot the results
% be aware of the variable names when using this code
% 
% posErr = xhatOut(1:2,:)-xTrue(1:2,:);
% posMSE = (sum(posErr(1,:).^2)+sum(posErr(2,:).^2))/size(posErr,2);

figure; title('Position estimation')
subplot(2,1,1);hold on;
plot(xTrue(1,:),'LineWidth',2);plot(xhatOut(1,:),'LineWidth',2);
legend('True x position','Estimated x position')
xlabel('simulation steps') 
ylabel('x position (m)') 

subplot(2,1,2);hold on;
plot(xTrue(2,:),'LineWidth',2);plot(xhatOut(2,:),'LineWidth',2);
legend('True y position','Estimated y position')
xlabel('simulation steps') 
ylabel('y position (m)') 


figure; title('Velocity estimation in x and y directions')
subplot(2,1,1);hold on;
plot(xTrue(3,:),'LineWidth',2);plot(xhatvel,'LineWidth',2);
legend('True x velocity','Estimated x velocity')
xlabel('simulation steps') 
ylabel('x velocity (m/s)') 

subplot(2,1,2);hold on;
plot(xTrue(4,:),'LineWidth',2);plot(yhatvel,'LineWidth',2);
legend('True y velocity','Estimated y velocity')
xlabel('simulation steps') 
ylabel('y velocity (m/s)') 
