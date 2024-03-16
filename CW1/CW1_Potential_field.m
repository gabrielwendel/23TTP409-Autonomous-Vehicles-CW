%% Course Work 1 Autonomous Vehicles 23TTP409 Loughborough University
% Potential field Path Planning
% Gabriel Wendel
close all;
clear all;
clc;

%% Parameters
% Vehicle starting position
x_vmin=5;
x_vmax=15;
y_vmin=10;
y_vmax=20;
x_veh=randi([x_vmin x_vmax]);
y_veh=randi([y_vmin y_vmax]);

%% Circle obstacle
xcmin=40;
xcmax=60;
ycmin=15;  
ycmax=55;
xc=randi([xcmin xcmax]);
yc=randi([ycmin ycmax]);
centers_circle=[xc yc]; % coordinate of the circle centre
radius=15; % radius

th = 0:pi/50:2*pi;
x_circle = (radius * cos(th) + xc);
y_circle = (radius * sin(th) + yc);
%% Pentagon obstacle
r_outer_pent=20;%outer radius covering the pentagon
t_pent=2*r_outer_pent*sind(36); % Side of the pentagon
xpcmax=105;
xpcmin=95;
ypcmax=105;
ypcmin=90;                           
xpc=randi([xpcmin xpcmax]); % center points for the pentagon
ypc=randi([ypcmin ypcmax]); %                                               
%Coordinates for the pentagon                    
xp1=xpc;  
yp1=r_outer_pent+ypc;
xp2=xpc+(r_outer_pent*cosd(18));
yp2=ypc+(r_outer_pent*sind(18));
xp3=xpc+(r_outer_pent*cosd(-54));
yp3=ypc+(r_outer_pent*sind(-54));
xp4=(xp3-t_pent);
yp4=yp3;
xp5=xpc-(r_outer_pent*cosd(18));
yp5=yp2;

xp = [xp1 xp2 xp3 xp4 xp5];
yp = [yp1 yp2 yp3 yp4 yp5];

% Generate coordinates along each side of the pentagon
N_points_per_side = 20; % Adjust the number of points

% Initialize arrays to store x and y coordinates
x_pentagon = zeros(1, N_points_per_side * 5);
y_pentagon = zeros(1, N_points_per_side * 5);

% Generate coordinates for each side using a loop
for i = 1:5
    x_pentagon((i-1)*N_points_per_side + 1 : i*N_points_per_side)...
        = linspace(xp(i), xp(mod(i,5)+1), N_points_per_side);
    y_pentagon((i-1)*N_points_per_side + 1 : i*N_points_per_side)...
        = linspace(yp(i), yp(mod(i,5)+1), N_points_per_side);
end
%% Hexagon obstacle
r_outer_hex=25;%outer radius covering the hexagon
% Hexagon outer circle radius= eaxh side of the hexagon
xhcmax=175;
xhcmin=140;
yhcmax=100;
yhcmin=70;
xhc=randi([xhcmin xhcmax]); % center points for hexagon
yhc=randi([yhcmin yhcmax]);
% Hexagon coordinates
xh1=xhc; 
yh1=r_outer_hex+yhc;
xh2=xhc+(r_outer_hex*cosd(30));
yh2=yhc+(r_outer_hex*sind(30));
xh3=xh2;
yh3=(yh2-r_outer_hex);
xh4=xhc+(r_outer_hex*cosd(-90));
yh4=yhc+(r_outer_hex*sind(-90));
xh5=-(-xhc+(r_outer_hex*cosd(-30)));
yh5=(yhc+(r_outer_hex*sind(-30)));
xh6=xh5;
yh6=yh5+r_outer_hex;

xh = [xh1 xh2 xh3 xh4 xh5 xh6];
yh = [yh1 yh2 yh3 yh4 yh5 yh6];

N_points_per_side = 20;

x_hexagon = zeros(1, N_points_per_side * 6);
y_hexagon = zeros(1, N_points_per_side * 6);

for i = 1:6
    x_hexagon((i-1)*N_points_per_side + 1 : i*N_points_per_side)...
        = linspace(xh(i), xh(mod(i,6)+1), N_points_per_side);
    y_hexagon((i-1)*N_points_per_side + 1 : i*N_points_per_side)...
        = linspace(yh(i), yh(mod(i,6)+1), N_points_per_side);
end
%% Triangle obstacle
radius_t= 20; % outer radius covering the triangle
xtcmax=50; 
xtcmin=25;
ytcmax=180;
ytcmin=130;
xtc=randi([xtcmin xtcmax]);
ytc=randi([ytcmin ytcmax]); % triangle center points
% Coordinates of the triangle
xt1=xtc;
yt1=radius_t+ytc;
xt2=radius_t*cosd(30)+xtc;
yt2=(-radius_t*sind(30)+ytc);
xt3=-radius_t*cosd(30)+xtc;
yt3=yt2;

xt = [xt1 xt2 xt3];
yt = [yt1 yt2 yt3];

N_points_per_side = 20;

x_triangle = zeros(1, N_points_per_side * 3);
y_triangle = zeros(1, N_points_per_side * 3);

for i = 1:3
    x_triangle((i-1)*N_points_per_side + 1 : i*N_points_per_side)...
        = linspace(xt(i), xt(mod(i,3)+1), N_points_per_side);
    y_triangle((i-1)*N_points_per_side + 1 : i*N_points_per_side)...
        = linspace(yt(i), yt(mod(i,3)+1), N_points_per_side);
end

Obs_x = [x_circle x_pentagon x_hexagon x_triangle]';
Obs_y = [y_circle y_pentagon y_hexagon y_triangle]';
Obs = [Obs_x, Obs_y];

%% Map
MapSize = 200; % 200 X 200 square
nObs = length(Obs);
Map = Obs';
hold on
% illustrare shape obstacles in different colors
plot(Obs(1:length(x_circle), 1), Obs(1:length(x_circle), 2), 'ro', 'LineWidth', 2, 'MarkerSize', 6);
plot(Obs(length(x_circle)+1:length(x_circle)+length(x_pentagon), 1), ...
    Obs(length(x_circle)+1:length(x_circle)+length(x_pentagon), 2),...
    'bo', 'LineWidth', 2, 'MarkerSize', 6);
plot(Obs(length(x_circle)+length(x_pentagon)+1:length(x_circle)+length(x_pentagon)+ ... 
    length(x_hexagon), 1), Obs(length(x_circle)+length(x_pentagon)+1:length(x_circle)+ ... 
    length(x_pentagon)+length(x_hexagon), 2), 'go', 'LineWidth', 2, 'MarkerSize', 6);
plot(Obs(length(x_circle)+length(x_pentagon)+length(x_hexagon)+1:end, 1), Obs(length(x_circle) ... 
    +length(x_pentagon)+length(x_hexagon)+1:end, 2), 'mo', 'LineWidth', 2, 'MarkerSize', 6);

axis equal


xlim([0 200])
ylim([0 200])

%% Potential Field Algorithm
% Based on code provided during tutorial

nMaxSteps = 800;

xGoal = [195; 195];
xStart = [x_veh; y_veh];
xVehicle = xStart;

RadiusOfInfluence = 100; % obstacle influence range

KGoal= 1; % attractive potential coefficient to the goal
KObj = 50; % repulsive potential coefficient to the goal

GoalError =  xGoal - xVehicle; % error vector

plot(xGoal(1),xGoal(2),'g*','MarkerSize', 6);


Hr = DrawRobot([xVehicle;0],'r',[]); % draw robot

k = 0;

pos = xStart; % store the trajectory in this array

while(norm(GoalError)>1 && k<nMaxSteps)
              
    
    % find distance to all obstacle entities
    % error vector between obstacles and the vehicle: q_obs - q
    Dp = Map-repmat(xVehicle,1,nObs); 
    Distance = sqrt(sum(Dp.^2)); 
    % determine which obstacles that influence vehicle
    iInfluencial = find(Distance<RadiusOfInfluence); 
    
    % if there are obstacles within influence range
    if(~isempty(iInfluencial))
        % vector sum of repulsions:
        rho = repmat(Distance(iInfluencial),2,1); %
        
        V = Dp(:,iInfluencial);
        
        DrhoDx = -V./rho;
        
        %         DrhoDx = -V;
        
        F = (1./rho-1./RadiusOfInfluence)*1./(rho.^2).*DrhoDx;
        
        FObjects = KObj*sum(F,2);    
        
    else
        % nothing close
        FObjects = [0;0];
    end

    % the gradient of the attractive potential is 
    FGoal = KGoal*(GoalError)/norm(GoalError); 
    % normalised FGoal = KGoal*(GoalError);
    
    % Combine attractive and repulsive forces to get the total force acting on the vehicle.
    FTotal = FGoal+FObjects;
    
    Magnitude = min(1,norm(FTotal));
    
    % Limit the magnitude of the total force to achieve smooth movement
    FTotal = FTotal/norm(FTotal)*Magnitude;
    
    % Update the vehicle position based on the total force
    xVehicle = xVehicle+FTotal;
    
    k = k+1;
    
    % compute the angle of the resultant force vector (FTotal), 
    % i.e. calculate the orientation that the vehicle should have based on the direction of
    % the total force acting on it.
    Theta = atan2(FTotal(2),FTotal(1));
    DrawRobot([xVehicle;Theta],'k',Hr);
    pause(0.0);
    drawnow;
    
    % Update error vector based on new position
    GoalError =  xGoal - xVehicle;

    % Save position in vector
    pos = [pos, xVehicle];
    
end

figure(1)
plot(pos(1,:),pos(2,:),'LineWidth', 2);
title('\textbf{Path Planned using Potential Field}','Interpreter','latex')
xlabel('x-coordinate','Interpreter','latex')
ylabel('y-coordinate','Interpreter','latex')
axis equal
grid on
xlim([0 200])
ylim([0 200])


%-------- Drawing Vehicle -----%
function H = DrawRobot(Xr,color,H)

p=0.02; % percentage of axes size 
a=axis;
l1=(a(2)-a(1))*p;
l2=(a(4)-a(3))*p;
P=[-1 1 0 -1; -1 -1 3 -1];%basic triangle
theta = Xr(3)-pi/2;%rotate to point along x axis (theta = 0)
c=cos(theta);
s=sin(theta);
P=[c -s; s c]*P; % rotate by theta
P(1,:)=P(1,:)*l1+Xr(1); % scale and shift to x
P(2,:)=P(2,:)*l2+Xr(2);
if(isempty(H))
    H = plot(P(1,:),P(2,:),color,'LineWidth',0.1);
else
    set(H,'XData',P(1,:));
    set(H,'YData',P(2,:));    
end
end
