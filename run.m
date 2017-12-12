%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
%
% Filename: run.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear, close all

% initial conditions
x1_0 = 0.3;                        % robot x position 
x2_0 = 2.1;                        % robot y position 
x3_0 = 0;                          % p
x4_0 = 1;                          % q
x5_0 = 0.1;                        % c

global co;                         % initial detection region radius
global r1;                         % obstacle x position 
global r2;                         % obstacle y position 
global bo;                         % obstacle ball radius
global x1t;                        % target x position
global x2t;                        % target y position

global num_obs;                    % total number of obstacles
global delta;                      % distance to tip of wedge
global doutside;
global voutside;
global gradoutside;
global mu;
global lambda;

% Scenario 1 - One obstacle
num_obs = 1;
r1(1) = 1;
r2(1) = 2.1;

% Scenario 2 - Two obstacles
% num_obs = 2;
% r1(1) = 1;
% r2(1) = 2.2;
% r1(2) = 2;
% r2(2) = 1.95;

% Scenario 3 - Detection region reduction
% num_obs = 2;
% r1(1) = 1;
% r2(1) = 2.45;
% r1(2) = 1;
% r2(2) = 1.75;
% r1(3) = 2.0;
% r2(3) = 2.45;
% r1(4) = 2.0;
% r2(4) = 1.75;    

% Scenario 4 - Second obstacle evasion when p = 1
% num_obs = 2;
% r1(1) = 1.35;
% r2(1) = 2.15;
% r1(2) = 0.95;
% r2(2) = 1.75;

% Scenario 5 - 4 obstacles, 4 starting conditions
% num_obs = 4;
% r1(1) = 1;
% r2(1) = 2.1;
% r1(2) = 1.6;
% r2(2) = 1.75;
% r1(3) = 1.6;
% r2(3) = 2.45;
% r1(4) = 2.2;
% r2(4) = 2.1;

% Scenario 6 - 4 obstacles, all pushing down
% num_obs = 4;
% r1(1) = 0.7;
% r2(1) = 2.1;
% r1(2) = 1.3;
% r2(2) = 2.0;
% r1(3) = 1.9;
% r2(3) = 1.9;
% r1(4) = 2.5;
% r2(4) = 1.85;

% Create your own scenario
% num_obs = 2;
% r1(1) = 1;
% r2(1) = 2.12;
% r1(2) = 1.6;
% r2(2) = 1.7;


co = 0.1;
delta = 0.09;
bo = 0.3;

% Target
x1t = 3.2;
x2t = 2.1;

doutside=0;
voutside=100;
gradoutside=10;
mu = 1.1;
lambda = 0.09;

x0 = [x1_0;x2_0;x3_0;x4_0;x5_0];

global axis_x;
global axis_y;
axis_x = 4;
axis_y = 2;

% simulation horizon
TSPAN=[0 20];
JSPAN = [0 20];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flowsd
rule = 1;

options = odeset('RelTol',1e-4,'MaxStep',.001);

% simulate
[t,j,x] = HyEQsolver( @fw,@g,@C,@D,...
    x0,TSPAN,JSPAN,rule,options);


plot_sim;