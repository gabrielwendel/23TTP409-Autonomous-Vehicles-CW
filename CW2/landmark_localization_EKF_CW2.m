% 23TTP409 coursework 2: EKF localisation 
% This script is to help you set up your algorithm for the landmark based
% localisation task

clear 
close all

%--------------------------------------------------------------------------
% simulation settings 

timeStep = 150; % total simulation steps
Ts=0.1; % sampling interval 

sig_bearing = 0.045; % standard deviation of bearing noise (rad)
sig_odometry = 0.7; % standard deviation of odometry noise (m/s)
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
% the states are x position, y position, x velocity and y velocity
xInit = [-30 -5 0 5]'; % initial state

% generate vehicle trajecotry using the unicycle vehicle model 
xpos = xInit(1); % x position 
ypos = xInit(2); % y position
psi = atan2(xInit(4),xInit(3)); % vehicle heading
vel = 5; % vehicle speed

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


%--------------------------------------------------------------------------


%==========================================================================
% Evaluation
%==========================================================================

% Note: to make sure the evaluation process can be carried out smoothly, 
% please put landmark_localization_EKF_eval.p in your current folder 


% uncomment the following three lines of code to evaluate of your code using another dataset

% clear xTrue landmarkMap zObv
% close all


% [xTrue, landmarkMap, zObv] = landmark_localization_EKF_eval(1234); % input the last four digits of your student number
%


%==========================================================================




%=========== Write your code here to process the sensor readings ==========

% xhatOut = EKF_landmark_localisation(zObv,landmarkMap, x0, P0, Q, R) 

% if possible, write your code as a Matlab function, where the arguments are:
% zObv: 2D matrix of sensor readings;
% landmarkMap: matrix of landmark locations
% x0: initial mean of the state distribution 
% p0: initial covariance matrix of the state distribution

% the function should output the estimated vehicle state to 'xhatOut'

% however, you can also write your EKF code as normal m-file code

% some coding suggestions:
% 1. Your code should contain a "for" loop to process sensor readings recursively
% 2. Choose the vehicle model for your EKF 
% 3. Define the sensor model that can incroprate three bearing readings w.r.t three landmarks  
% 4. However, it might be useful to use only one or two landmark readings at the beginning
% 5. Be careful when comparing bearing angle differences
% 6. Tune your parameters to improve the performance 

% Initial state estimate and covariance matrix
x0 = [-30; -5; 0; 5; 5; 0]; % Initial state [x, y, v, psi, v, omg]
P0 = eye(6); % Initial covariance matrix

% Process noise covariance Q initialization (ensure it's 6x6)
Q = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]); % Diagonal covariance matrix

% Define the state transition matrix G
G = eye(6); % Identity matrix for simplicity
G(1:2, 3:4) = Ts * eye(2); % Fill in the appropriate values


% Call the EKF function
xhatOut = EKF_landmark_localisation(zObv, landmarkMap, x0, P0, Q, R);



%==========================================================================

%% postprocess of the results 
% uncomment to use the code to plot the results
% be aware of the variable names when using this code

% posErr = xhatOut(1:2,:)-xTrue(1:2,:);
% posMSE = (sum(posErr(1,:).^2)+sum(posErr(2,:).^2))/size(posErr,2);
% 
% figure; title('Position estimation')
% subplot(2,1,1);hold on;
% plot(xTrue(1,:),'LineWidth',2);plot(xhatOut(1,:),'LineWidth',2);
% legend('True x position','Estimated x position')
% xlabel('simulation steps') 
% ylabel('x position (m)') 
% 
% subplot(2,1,2);hold on;
% plot(xTrue(2,:),'LineWidth',2);plot(xhatOut(2,:),'LineWidth',2);
% legend('True y position','Estimated y position')
% xlabel('simulation steps') 
% ylabel('y position (m)') 
% 
% 
% figure; title('Velocity estimation in x and y directions')
% subplot(2,1,1);hold on;
% plot(xTrue(3,:),'LineWidth',2);plot(xhatOut(3,:),'LineWidth',2);
% legend('True x velocity','Estimated x velocity')
% xlabel('simulation steps') 
% ylabel('x velocity (m/s)') 
% 
% subplot(2,1,2);hold on;
% plot(xTrue(4,:),'LineWidth',2);plot(xhatOut(4,:),'LineWidth',2);
% legend('True y velocity','Estimated y velocity')
% xlabel('simulation steps') 
% ylabel('y velocity (m/s)') 

% Note that you can also compare the longitudinal velocity directly

%%

function xhatOut = EKF_landmark_localisation(zObv, landmarkMap, x0, P0, Q, R)
    % Initialize variables
    xhat = x0; % Initial state estimate
    Phat = P0; % Initial covariance matrix
    nLandmark = size(landmarkMap, 2); % Number of landmarks

    % Motion model parameters
    Ts = 0.1; % Sampling interval

    % Define the bicycle model for the EKF
    % State vector: [x, y, v, psi]
    % Control vector: [v, omg]
    % Measurement vector: [bearing1; bearing2; bearing3]
    % Measurement function h(x) = atan2(landmark_y - y, landmark_x - x) + psi
    % Jacobian H = dh/dx
    % Measurement noise covariance R
    % Process noise covariance Q

    % Process noise covariance matrix
    G = eye(4);
    G(1:2, 3:4) = Ts * eye(2);
    Q = G * Q * G';

    xhatOut = zeros(4, size(zObv, 2)); % Estimated vehicle state trajectory

    % EKF loop
    for t = 1:size(zObv, 2)
        % Prediction Step: Predict the next state
        v = xhat(3);
        omg = xhat(6);
        psi = xhat(4);

        % Motion model (bicycle model)
        F = eye(4);
        F(1, 3) = Ts * cos(psi);
        F(2, 3) = Ts * sin(psi);

        xhat_minus = xhat + Ts * [v * cos(psi); v * sin(psi); 0; omg; 0; 0];
        Phat_minus = F * Phat * F' + Q;

        % Update Step: Incorporate sensor measurements
        H = zeros(nLandmark, 4);
        zhat = zeros(nLandmark, 1);
        Rk = zeros(nLandmark);

        for k = 1:nLandmark
            % Measurement function h(x)
            xLandmark = landmarkMap(1, k);
            yLandmark = landmarkMap(2, k);
            dx = xLandmark - xhat_minus(1);
            dy = yLandmark - xhat_minus(2);
            q = dx^2 + dy^2;
            zhat(k) = atan2(dy, dx) - xhat_minus(4); % Predicted bearing

            % Jacobian H
            H(k, 1) = -dy / q;
            H(k, 2) = dx / q;
            H(k, 4) = -1;

            % Measurement noise covariance
            Rk(k, k) = R;
        end

        % Kalman gain
        K = Phat_minus * H' / (H * Phat_minus * H' + Rk);

        % Measurement update
        innovation = zObv(:, t) - zhat;
        innovation = wrapToPi(innovation); % Ensure angles are within [-pi, pi]
        xhat = xhat_minus + K * innovation;
        Phat = (eye(4) - K * H) * Phat_minus;

        % Store estimated state
        xhatOut(:, t) = xhat;
    end
end

