%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file              Author: Ricardo Sanfelice, Vijay Muthukumaran
%
% Project 1: Simulation of problem on target acquisition and obstacle
% avoidance.
% Project 2: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: plotLyapunov2
%
% Description: plot Lyapunov function
%
% Version: 1
% Required files: LyapunovFunction.m, dist_circle.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mStep = 0.01;
axis_x = 4;
axis_y = 2;
%[a,b] = meshgrid(-1:mStep:axis_x,(axis_y-2):mStep:(axis_y+2));
%[N,M] = size(a);
for i=1:length(x(:,1))
        v(i) = LyapunovFunction3([x(i,1),x(i,2)],r1,r2,num_obs,control_space,doutside,voutside,x1t,x2t);
end

figure(2)
plot(v(1:1500))
grid on;
xlabel('t')
ylabel('V_{q}')
