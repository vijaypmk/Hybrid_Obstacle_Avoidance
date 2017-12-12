%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file               Author: Ricardo Sanfelice, Vijay Muthukumaran
%
% Project 1: Simulation of problem on target acquisition and obstacle
% avoidance.
% Project 2: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: plotLyapunovXYZ
%
% Description: plot Lyapunov function as mesh
%
% Version: 1
% Required files: LyapunovFunction.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mStep = 0.01;
[a,b] = meshgrid(-1:mStep:4,-2:mStep:2);
[N,M] = size(a);
for i=1:N,
   for j=1:M,
       v(i,j) = LyapunovFunction([a(i,j),b(i,j)],r1,r2,num_obs,control_space,doutside,voutside,x1t,x2t);
   end
end

figure(1),clf
meshc(a,b,v);
%[cs,h] = contour(a,b,v,20);
%figure(2),clf
%mesh(a,b,v);

%clabel(cs,h,'labelspacing',72)
