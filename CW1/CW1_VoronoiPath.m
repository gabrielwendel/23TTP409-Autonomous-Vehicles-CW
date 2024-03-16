%% Course Work 1 Autonomous Vehicles 23TTP409 Loughborough University
% Voronoi Diagram Path Planning
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
veh_pos0 = [x_veh y_veh 0]; % initial x,y,z position of vehicle

%% Obstacles
% Four obstacles: one circle, one pentagon, one hexagon and one triangle

%% Circle
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

%% Pentagon
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
N_points_per_side = 10; % Adjust the number of points as needed

% Initialize arrays to store x and y coordinates
x_pentagon = zeros(1, N_points_per_side * 5);
y_pentagon = zeros(1, N_points_per_side * 5);

% Generate coordinates for each side using a loop
for i = 1:5
    x_pentagon((i-1)*N_points_per_side + 1 : i*N_points_per_side) = linspace(xp(i), xp(mod(i,5)+1), N_points_per_side);
    y_pentagon((i-1)*N_points_per_side + 1 : i*N_points_per_side) = linspace(yp(i), yp(mod(i,5)+1), N_points_per_side);
end

% 
% x_pentagon=[xp1 xp2 xp3 xp4 xp5]; % x_coordinate of the pentagon vertices
% y_pentagon=[yp1 yp2 yp3 yp4 yp5]; % y_coordinate of the pentagon vertices

%% Hexagon
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

% Generate coordinates along each side of the hexagon
N_points_per_side = 10; % Adjust the number of points as needed

% Initialize arrays to store x and y coordinates
x_hexagon = zeros(1, N_points_per_side * 6);
y_hexagon = zeros(1, N_points_per_side * 6);

% Generate coordinates for each side using a loop
for i = 1:6
    x_hexagon((i-1)*N_points_per_side + 1 : i*N_points_per_side) = linspace(xh(i), xh(mod(i,6)+1), N_points_per_side);
    y_hexagon((i-1)*N_points_per_side + 1 : i*N_points_per_side) = linspace(yh(i), yh(mod(i,6)+1), N_points_per_side);
end

% x_hexagon=[xh1 xh2 xh3 xh4 xh5 xh6]; % x_coordinate of the hexagon vertices
% y_hexagon=[yh1 yh2 yh3 yh4 yh5 yh6]; % y_coordinate of the hexagon vertices

%% Triangle
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

% Generate data points forming the sides of the triangle
N_points_per_side = 10; % Adjust the number of points as needed

% Initialize arrays to store x and y coordinates
x_triangle = zeros(1, N_points_per_side * 3);
y_triangle = zeros(1, N_points_per_side * 3);

% Generate coordinates for each side using a loop
for i = 1:3
    x_triangle((i-1)*N_points_per_side + 1 : i*N_points_per_side) = linspace(xt(i), xt(mod(i,3)+1), N_points_per_side);
    y_triangle((i-1)*N_points_per_side + 1 : i*N_points_per_side) = linspace(yt(i), yt(mod(i,3)+1), N_points_per_side);
end

Obs_x = [x_circle x_pentagon x_hexagon x_triangle]';
Obs_y = [y_circle y_pentagon y_hexagon y_triangle]';
Obs = [Obs_x, Obs_y];

%% set up the map
[Vx,Vy] = voronoi(Obs(:,1),Obs(:,2));

hold on
plot(Vx,Vy,'b-',Obs(:,1),Obs(:,2),'ro','LineWidth', 2,'MarkerSize', 6);
axis equal

% Configuration space
xlim([0 200]);
ylim([0 200]);

shape_x_coord = {xp, xh, xt};
shape_y_coord = {yp, yh, yt};

num_shapes = 3; % Number of shapes

%% Compute Voronoi Diagram

dt = delaunayTriangulation(Obs);
[V,R] = voronoiDiagram(dt);

% Assign labels to the Voronoi vertices V.
% By convention the first vertex is at infinity.
numv = size(V,1);

% Construct the adjacency matrix
A = zeros(numv+1, numv+1);

% Find vertices inside each obstacle
inside_pentagon = inpolygon(V(:,1), V(:,2), xp, yp);
inside_hexagon = inpolygon(V(:,1), V(:,2), xh, yh);
inside_triangle = inpolygon(V(:,1), V(:,2), xt, yt);

for ii = 1:length(R)
    cell = R{ii};
    for jj = 1:length(cell)
        
        if jj == length(cell)
            A(cell(jj),cell(1)) = 1;
        else
            A(cell(jj),cell(jj+1)) = 1;
        end        
    end 
end


% add the start point and end point
Start = [x_veh, y_veh];
End = [195, 195];

% find the closest vertex or node (find the index for this node): 

for i=1:length(V)-1
    distances(i) = norm(V(i+1,:) - Start);
end

min_dist = min(distances);
min_index = find(distances==min_dist);

% find the closest node for the end point (find the index for this node):

for i=1:length(V)-1
    distances_end_point(i) = norm(V(i+1,:) - End);
end

min_dist_end = min(distances_end_point);
min_index_end = find(distances_end_point==min_dist_end);

% the start point and end point are added into the graph (already done for you)
V(1,:) = Start; % replace the first node as start point
V = [V; End]; % add the end point as the last vertex

% Modify the adjacency matrix by adding connections to their closest
% points, respectively. 
A(1,:) = 0;
A(:,1) = 0;

A(1,min_index) = 1;
A(min_index,1) = 1;

A(end,min_index_end) = 1;
A(min_index_end,end) = 1;

%% Create Graph
G = graph(A);

% Calculate edge distances
N_edge = numedges(G);
[sOut,tOut] = findedge(G);

edgeDist = sqrt(sum((V(sOut,:)-V(tOut,:)).^2,2));
G.Edges.Weight = edgeDist;

% plot noded and edged of the graph
pG = plot(G,'XData', V(:,1), 'YData', V(:,2), 'LineWidth', 1, 'MarkerSize', 2);
plot(Start(1), Start(2),'k*'); plot(End(1), End(2),'r*');


% find the shortest path using Dijkstra 
[path, dist_path] = shortestpath(G,1,length(V)); 
highlight(pG,path,'EdgeColor','k','LineWidth', 6)
