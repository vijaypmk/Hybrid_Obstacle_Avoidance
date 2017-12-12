function [in_wedge, d_wedge, grad_x1, grad_x2] = in_wedge_2(x,r1,r2,p_obs,bo,control_space)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: dist_circle.m
%
% Description: Computes obstacle ball detection with detection region and
% returns obstacle position(obs_pos) and number of obstacles
% detected(obs_num)
%
% Version: 2
% Required files: -
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = x(1);
x2 = x(2);

p1 = r1(p_obs) + 0.3;
p2 = r2(p_obs) - 0.3;
rx1 = r1(p_obs) - 0.2:0.01:p1;
rx2 = p2:0.01:r2(p_obs) + 0.2;
% lx2 = -lx1 + p2;

in_wedge = 0;
d_wedge = 1000;

for i = 1:length(rx2)
%     if (x2 <= lx2(i)) && (x1 <= lx1(i))
    if x2 <= rx1(i) + r2(p_obs) - 0.09 && x1 >= r1(p_obs)
        in_wedge = 1;
        d_temp = sqrt((x2 - rx2(i))^2 + (x1 - rx1(i))^2);
        gx1 = (x1 - rx1(i))/sqrt((x2 - rx2(i))^2 + (x1 - rx1(i))^2);
        gx2 = (x2 - rx2(i))/sqrt((x2 - rx2(i))^2 + (x1 - rx1(i))^2);
%         d_temp = (rx1(i) - r1(p_obs) + r2(p_obs) - rx2(i) - control_space)/(sqrt(2));
        if (d_temp < d_wedge)
            d_wedge = d_temp;
            grad_x1 = gx1;
            grad_x2 = gx2;
        end
    else
        in_wedge = 0;
        d_wedge = 1000;
        grad_x1 = 0;
        grad_x2 = 0;
    end
end
end