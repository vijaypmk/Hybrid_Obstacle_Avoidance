%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: C.m
%
% Description: Flow Set
%
% Version: 9
% Required files: dist_circle.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [value] = C(x) 

x1 = x(1);
x2 = x(2);
p = x(3);
q = x(4);
c = x(5);

global r1;
global r2;
global num_obs;
global bo;
global co;
global doutside;
global voutside;
global x1t;
global x2t;
global mu;
global lambda;


% with current value of c
[d_obs, p_obs, obs_num] = dist_circle([x1,x2],r1,r2,num_obs,bo,c);
% with original value of c
[d_obs_o, p_obs_o, obs_num_o] = dist_circle([x1,x2],r1,r2,num_obs,bo,co+.05);


if obs_num == 0
    if obs_num_o == 0      
        if p > 0
            value = 0;          % Obstacle evaded
        else
            value = 1;          % C1
        end
    else
        value = 1;              % C3
    end
else
    if p == 0 || p > 1     
        value = 0;              % Obstacle detected
    else
        v1 = LyapunovFunction1([x1,x2],r1,r2,p_obs,c,doutside,voutside,x1t,x2t);
        v2 = LyapunovFunction2([x1,x2],r1,r2,p_obs,c,doutside,voutside,x1t,x2t);
        vmin = min(v1,v2);

        if v1 >= (mu - lambda)*vmin && q == 1
            value = 0;
        elseif v2 > (mu - lambda)*vmin && q == 2
            value = 0;
        else
            value = 1;              % C2
        end
    end
end


end