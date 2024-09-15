% NameFile: MAIN_Workspace_2R
% Copyright: Alessio Borgi
% Contacts: borgi.1952442@studenti.uniroma1.it
% Date: 18-03-2024
% Course: Robotics 1
% Degree: Master's Degree in Artificial Intelligence and Robotics 
% Version: v1.0

%%%%%% TASK %%%%%%

%   Given:
%   - DH1: This is a vector of DH parameters, for each link.
%   - q: This is a vector of Joint Limits.

%   Find: 
%   - Workspace of a 2R Planar Arm.

%%%%%% END TASK %%%%%%

%%%%%% PARAMETERS TO SET %%%%%%
clc

% Define DH parameters for each link.
a1 = 1; 
a2 = 0.5; 
DH1(1) = Link([0 0 a1 0]);
DH1(2) = Link([0 0 a2 0]);

% Define Joint Angles Ranges.
th1 = (0 : 0.01 : pi/2);
th2 = (-pi/2 : 0.01 : pi/2);
q = {th1,th2};

%%%%%% END PARAMETERS %%%%%%


%%%%%% START PROGRAM %%%%%%

% Define the colors.
orange = [0.8500 0.3250 0.0980];
blue = [0 0.4470 0.7410];
light_blue = [0.3010 0.7450 0.9330];
yellow = [0.9290 0.6940 0.1250];
green = [0 1 0];
red = [1 0 0];
black = [0 0 0];

% Plot workspace with different constraints and colors.

% Full mobility of link2 and link1 still at 0.
Workspace(DH1, {(0),(-pi/2:0.01:pi/2)}, light_blue) 
% Full mobility of link1 and link1 still at pi/2.
Workspace(DH1, {(0:0.01:pi/2),(pi/2)}, yellow) 
% Full mobility of link1 and link1 still at 0.
Workspace(DH1, {(0:0.01:pi/2),(0)}, green) 
% Full mobility of link1 and link1 still at -pi/2.
Workspace(DH1, {(0:0.01:pi/2),(-pi/2)}, orange) 
% Full mobility of link2 and link1 still at pi/2.
Workspace(DH1, {(pi/2),(-pi/2:0.01:pi/2)}, blue) 

% Plot the a point in order to check whether it is inside WS1.
x = 1.6;
y = -0.2;
plot(x,y,'MarkerEdge',black, 'LineWidth', 2, 'Marker','*', 'MarkerSize', 25)

% Plot Link 1 boundary. 
r= a1 ;
teta=-pi:0.01:pi;
x=r*cos(teta);
y=r*sin(teta);
plot(x,y,"mo", 'LineWidth', 0.6)

% Plot Outer boundary.
r= a1 + a2 ;
teta=-pi:0.01:pi;
x=r*cos(teta);
y=r*sin(teta);
plot(x,y,"k-", 'LineWidth', 0.6)

% Plot Inner boundary.
r= sqrt(a1^2 + a2^2) ;
teta=-pi:0.01:pi;
x=r*cos(teta);
y=r*sin(teta);
plot(x,y,"k-", 'LineWidth', 0.6)

% Plot horizontal and vertical lines.
yline(0)
xline(0)

% Plot WS2
r= abs(a2 - a1);
teta=-pi:0.01:pi;
x=r*cos(teta);
y=r*sin(teta);
plot(x,y,'MarkerFace',red, 'LineWidth', 2, 'Marker','.', 'MarkerSize', 25)

hold off

%%%%%% END PROGRAM %%%%%%