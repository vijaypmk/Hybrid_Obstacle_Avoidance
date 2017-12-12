function [d] = Distance1(x,r1,r2,p_obs,delta,control_space,doutside) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Simulation of problem on target acquisition and obstacle
% avoidance.
% 
% Name: Distance1
%
% Description: Computes the distance to outside of region O1
%
% Version: 1
% Required files: -
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1= x(1);
x2= x(2);
i = p_obs;

if(x1-r1(i))^2+(x2-r2(i))^2> (delta^2/8 + control_space^2/8)
    if ((x2 -r2(i)) + delta + control_space < - x1 + r1(i)) && ( x1 - r1(i) < (x2-r2(i)) + delta + control_space)
        d = (r1(i) + r2(i) - x1 - x2 - delta - control_space)/(sqrt(2));
    elseif ((x2-r2(i)) + delta + control_space > -x1 + r1(i)) && (x1 - r1(i) > (x2-r2(i)) + delta + control_space)
        d = (x1 - r1(i) + r2(i) - x2 - delta - control_space)/(sqrt(2));
    else
          d = sqrt((x1-r1(i))^2+(x2-r2(i))^2)- (delta + control_space);
    end       
else 
    d = doutside;
end
end
    
