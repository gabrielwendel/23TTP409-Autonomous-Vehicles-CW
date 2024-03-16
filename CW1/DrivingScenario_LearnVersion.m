%% Creating Driving Scenario
scenario = drivingScenario;

roadCenters = [100 0; 100 200];
roadWidth = 200;
road(scenario, roadCenters, roadWidth);

%%  Vehicle Initial states
x_vmin=5;
x_vmax=15;
y_vmin=10;
y_vmax=20;
x_veh=randi([x_vmin x_vmax]);
y_veh=randi([y_vmin y_vmax]);
veh_pos0 = [x_veh y_veh 0]; % initial x,y,z position of vehicle
veh_v0= 8; % initial vehicle velocity m/s;
veh_yaw0 = 90*rand(1); % heading angle;
veh_vel0=[veh_v0*cos(veh_yaw0) veh_v0*cos(veh_yaw0) 0]; % Initial velocity along x,y,z
egoVehicle=vehicle(scenario,'Position',veh_pos0,'Velocity',veh_vel0,'Yaw',veh_yaw0);                                                       
plot(scenario);
set(gcf,'Name','Scenario Plot')
hold on;

 %%
% Once all the actors in a scenario have been created, you can inspect the
% pose information of all the actors in the coordinates of the scenario by
% inspecting the |Position|, |Roll|, |Pitch|, |Yaw|, |Velocity|, and
% |AngularVelocity| properties of each actor, or you may obtain all of them
% in a convenient structure by calling the |actorPoses| method of the
% scenario:

ap = actorPoses(scenario);

%% Start Position and Goal Postion Coordinates
 plot(195,195,'r.','MarkerSize',40); hold on
 txt = ['Goal'];
text(199,199,txt,'fontsize',12)
 plot(x_veh,y_veh,'g.','MarkerSize',40);hold on
 txt = ['Start'];
text((x_veh-2),(y_veh-2),txt,'fontsize',12)

%% Defining the obstacles
%% First obstacle coordinates (circle)

xcmin=40; %
xcmax=60; %
ycmin=15; %  
ycmax=55; %
xc=randi([xcmin xcmax]);
yc=randi([ycmin ycmax]);
centers_circle=[xc yc]; % coordinate of the circle centre
radius=15; % radius

viscircles(centers_circle,radius,'color',[1 0.5 0.2]);
hold on

th = 0:pi/50:2*pi;
x_circle = radius * cos(th) + xc;
y_circle = radius * sin(th) + yc;
fill(x_circle, y_circle, [1 0.5 0.2])
hold on

%% Second Obstacle (Pentagon)

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
x_pentagon=[xp1 xp2 xp3 xp4 xp5 xp1]; % x_coordinate of the pentagon vertices
y_pentagon=[yp1 yp2 yp3 yp4 yp5 yp1]; % y_coordinate of the pentagon vertices
plot(x_pentagon,y_pentagon)
fill(x_pentagon,y_pentagon,'b')
hold on

%% Third obstacle Hexagon

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
x_hexagon=[xh1 xh2 xh3 xh4 xh5 xh6 xh1]; % x_coordinate of the hexagon vertices
y_hexagon=[yh1 yh2 yh3 yh4 yh5 yh6 yh1]; % y_coordinate of the hexagon vertices
plot(x_hexagon,y_hexagon);
fill(x_hexagon,y_hexagon,'g')
hold on

%% Fourth obstacle (triangle)

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

xt=[xt1 xt2 xt3 xt1]; % x_coordinate of the hexagon vertices
yt=[yt1 yt2 yt3 yt1]; % y_coordinate of the hexagon vertices
plot(xt,yt);
fill(xt,yt,'m')
hold on

%% Simulation

% inital states of the vehicle
x_pos = veh_pos0(1);
y_pos = veh_pos0(2);
theta = deg2rad(veh_yaw0);
vel = veh_v0;
delT = 0.05;

flag = 0;

for ii = 1:500
  
    %==== Vehicle Model====================================================
    % Do not change
    % 
    if flag > 1
        flag = 1;
    end
    if flag < -1
        flag = -1;
    end
    x_pos = x_pos + vel*cos(theta)*delT;
    y_pos = y_pos + vel*sin(theta)*delT;
    theta = theta + flag*delT;
    %======================================================================
    
    
    egoVehicle.Position = [x_pos y_pos 0];
    egoVehicle.Yaw = rad2deg(theta);
    updatePlots(scenario);
    
    pause(0.1);
end  