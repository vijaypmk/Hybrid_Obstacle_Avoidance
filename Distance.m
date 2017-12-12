function [d] = Distance(x,r1,r2,num_obs,control_space,doutside) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Ricardo Sanfelice, Vijay Muthukumaran
%
% Project 1: Simulation of problem on target acquisition and obstacle
% avoidance.
% Project 2: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: Distance
%
% Description: Computes the distance to the obstacle (circle).
%
% Version: 1
% Required files: -
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1= x(1);
x2= x(2);
i = num_obs;

if (x1-r1(i))^2+(x2-r2(i))^2> (control_space^2/8)
    d = sqrt((x1-r1(i))^2+(x2-r2(i))^2)- (control_space);
else 
    d = doutside;
end
end
    
