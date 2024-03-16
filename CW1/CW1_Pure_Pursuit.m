%% Course Work 1 Autonomous Vehicles 23TTP409 Loughborough University
% Pure Pursuit Path Planning
% Gabriel Wendel

% reference points
refPose = pos;
xRef = refPose(1,:)';
yRef = refPose(2,:)';

Ts = 40; % simulation time
L = 3; % bicycle length
ld = 5; % lookahead distance
X_o = refPose(1,1); % initial vehicle position
Y_o = refPose(1,2); % initial vehicle position 
psi_o = 90*rand(1); % initial yaw angle
veh_v0= 8; % initial vehicle velocity m/s;

% paramters from Simulink
SimOut = sim('CW1_Pure_Pursuit_sim.slx');

figure(2)
hold on

% Plot the planned path
plot(xRef, yRef, 'LineWidth', 3)

% Plot the vehicle path with red circles
plot(SimOut.real_x_pos(:,2), SimOut.real_y_pos(:,2), '--', 'LineWidth', 1)

legend('Planned Path', 'Vehicle Path')
title('\textbf{Path followed by the vehicle and planned path}','Interpreter','latex');
xlabel('x-coordinate','Interpreter','latex')
ylabel('y-coordinate','Interpreter','latex')
xlim([0 200])
ylim([0 200])
grid on
hold off
