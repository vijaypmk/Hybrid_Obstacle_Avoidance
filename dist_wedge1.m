function [d, n, gx1, gx2] = dist_wedge1(x,r1,r2,p_obs,control_space,doutside,q) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: dist_wedge1.m
%
% Description: Computes the distance to outside of wedges
%
% Version: 1
% Required files: -
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1= x(1);
x2= x(2);

if(x1-r1(p_obs))^2+(x2-r2(p_obs))^2 <= (.3 + control_space)
        [iw, dw, g11, g12] = in_wedge_1([x1, x2], r1, r2, p_obs, .3, control_space);
        [iw2, dw2, g21, g22] = in_wedge_2([x1, x2], r1, r2, p_obs, .3, control_space);
        if (iw)
          d = dw;
          n = 1;
          gx1 = g11;
          gx2 = g12;
        elseif (iw2)
          d = dw2;
          n = 2;
          gx1 = g21;
          gx2 = g22;
        else
            d = sqrt((x1-r1(p_obs))^2+(x2-r2(p_obs))^2)- (control_space);
        end
else
    d = 1;
    n = 1;
    gx1 = 0;
    gx2 = 0;
end

end
    
