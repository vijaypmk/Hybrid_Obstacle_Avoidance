function [v] = LyapunovFunction2(x,r1,r2,p_obs,control_space,doutside,voutside,x1t,x2t) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file             Author: Ricardo Sanfelice, Vijay Muthukumaran
%
% Project 1: Simulation of problem on target acquisition and obstacle
% avoidance.
% Project 2: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
%
%
% Name: LyapunovFunction2
%
% Description: Computes the Lyapunov function for the gradient
% control case. 
%
% Version: 1
% Required files: Distance1.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x1t and x2t are the target coordinates

x1= x(1);
x2= x(2);

v = 10/2*(x1-x1t)^2+10/2*(x2-x2t)^2;
i = p_obs;
d = Distance1([x1,-x2],r1,-r2,i,.3,control_space,doutside);
if (d <= 1) && (d >0)
    v = v+(d-1)^2*log(1/d);
elseif (d>1)
    v = v;
else
    v = voutside;
end
end
