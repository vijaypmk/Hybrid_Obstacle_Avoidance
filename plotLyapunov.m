%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file              Author: Ricardo Sanfelice, Vijay Muthukumaran
%
% Project 1: Simulation of problem on target acquisition and obstacle
% avoidance.
% Project 2: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: plotLyapunov1
%
% Description: plot Lyapunov function
%
% Version: 1
% Required files: LyapunovFunction.m, dist_circle.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mStep = 0.01;
axis_x = 4;
axis_y = 2;
[a,b] = meshgrid(-1:mStep:axis_x,(axis_y-2):mStep:(axis_y+2));
[N,M] = size(a);
for i=1:N,
    for j=1:M,
        v(i,j) = LyapunovFunction([a(i,j),b(i,j)],r1,r2,num_obs,co,doutside,voutside,x1t,x2t);
        [d, p] = dist_circle([a(i,j),b(i,j)],r1,r2,num_obs,bo,co);
        % for computing vector fields
%         for k=1:length(p)
%         if(~d(k))
%             tgradVx1(i,j) = 10*(a(i,j)-x1t);
%             tgradVx2(i,j) = 10*(b(i,j)-x2t);
%         else
%             tgradVx1(i,j) = 5*(a(i,j)-r1(p(k))-x1t) + (25/sqrt((a(i,j) - r1(p(k)))^2 + (b(i,j) - r2(p(k)))^2) - bo) * (b(i,j) - r2(p(k)));
%             tgradVx2(i,j) = 5*(b(i,j)+r2(p(k))-x2t) - (25/sqrt((a(i,j) - r1(p(k)))^2 + (b(i,j) - r2(p(k)))^2) - bo) * (a(i,j) - r1(p(k)));
% %             tgradVx1(i,j) = 10*(a(i,j)-x1t) + (50/sqrt(2)) * (b(i,j) - r2(1));
% %             tgradVx2(i,j) = 10*(b(i,j)-x2t) - (50/sqrt(2)) * (a(i,j) - r1(1));
%         end
%         end
    end
end

% [Dx,Dy] = gradient(v,10,10);

figure(1)
[cs,h] = contour(a,b,v,20);
% hold on, quiver(a1,b1,-tgradVx1,-tgradVx2);
% for k = 1:length(p)
%     viscircles([r1(p(k)) r2(p(k))], bo,'LineStyle','--');
% end
% figure(2),clf
% mesh(a,b,v);

%clabel(cs,h,'labelspacing',72)
