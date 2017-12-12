%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: fw.m
%
% Description: Flow map with wedges
%
% Version: 20
% Required files: dist_circle, dist_wedge1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xdot = fw(x)
    
% state
x1 = x(1);
x2 = x(2);
p = x(3);
q = x(4);
c = x(5);

global r1;
global r2;
global x1t;
global x2t;
global delta;
global num_obs;
global doutside;
global gradoutside;
global bo;
global d1;
global d2;


[d_obs, p_obs, obs_num] = dist_circle([x1,x2],r1,r2,num_obs,bo,c);


if obs_num == 0
    gradVx1 = 10*(x1-x1t);
    gradVx2 = 10*(x2-x2t);
else
    if q == 1
        [d1, nw, gx1, gx2] = dist_wedge1([x1,x2],r1,r2,p_obs,c,doutside,q);
        if nw == 1
            if d1 <= delta
                gradDx1 = gx1;
                gradDx2 = gx2;
            else
                gradDx1 = gradoutside;
                gradDx2 = gradoutside;
            end
        else
            gradDx1 = gradoutside;
            gradDx2 = gradoutside;
        end
        if d1 <= delta && nw == 1
            gradVx1 = 15*(x1-r1(p_obs)-x1t) + 10*(d1-1)*gradDx1*log(1/d1)-(d1-1)^2 * gradDx1 / d1;
            gradVx2 = 15*(x2+r2(p_obs)-x2t) + 10*(d1-1)*gradDx2*log(1/d1)-(d1-1)^2 * gradDx2 / d1;
        else
            gradVx1 = 10*(x1-x1t);
            gradVx2 = 10*(x2-x2t);
        end
    else
        x2m = -x2;
        r2m = -r2(p_obs);
        [d2, nw, gx1, gx2] = dist_wedge1([x1,-x2],r1,-r2,p_obs,c,doutside,q);
        if nw == 2
            if d2 < delta
                gradDx1 = gx1;
                gradDx2 = gx2;
            else
                gradDx1 = gradoutside;
                gradDx2 = gradoutside;
            end
        else
            gradDx1 = gradoutside;
            gradDx2 = gradoutside;
        end
        if d2 < delta && nw == 1
            gradVx1 = 15*(x1-r1(p_obs)-x1t) + 10*(d2-1)*gradDx1*log(1/d2)-(d2-1)^2 * gradDx1 / d2;
            gradVx2 = 15*(x2-r2(p_obs)-x2t) + 10*(d2-1)*gradDx2*log(1/d2)-(d2-1)^2 * gradDx2 / d2;
        else
            gradVx1 = 10*(x1-x1t);
            gradVx2 = 10*(x2-x2t);
        end
    end
end

% update control using direction of gradient to speed things up
dir2 = atan(gradVx2/gradVx1);
if (x1 == x1t && x2 == x2t) || (round(gradVx1) == 0 && round(gradVx2) == 0)
    ucontrol = [0;0];
else
    ucontrol = [1*30*cos(dir2);1*30*sin(dir2)];
    % To use original grad, uncomment below
    %ucontrol = [-gradVx1; -gradVx2];
end

xdot = [ucontrol; 0; 0; 0];
end