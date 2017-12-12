%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Vijay Muthukumaran
%
% Project: Robot Motion Planning with Avoidance of Multiple Obstacles Using
% Potential Field and Hybrid Controller
% 
% Name: g.m
%
% Description: Jump Map
%
% Version: 10
% Required files: dist_circle, dist_wedge1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xplus = g(x)

% state
x1 = x(1);
x2 = x(2);
p = x(3);
q = x(4);
c = x(5);

global num_obs;
global doutside;
global r1;
global r2;
global bo;
global co;
global voutside;
global x1t;
global x2t;
global mu;
global lambda;


[d_obs, p_obs, obs_num] = dist_circle([x1,x2],r1,r2,num_obs,bo,c);
p = obs_num;
eta = 0.9;

if p == 1
    v1 = LyapunovFunction1([x1,x2],r1,r2,p_obs,c,doutside,voutside,x1t,x2t);
    v2 = LyapunovFunction2([x1,x2],r1,r2,p_obs,c,doutside,voutside,x1t,x2t);
    vmin = min(v1, v2);

    if v1 <= v2
        q = 1;
    else 
        q = 2;
    end

    if q == 1 && (v1 >= (mu - lambda) * vmin)
        q = 2;
    elseif q == 2 && (v2 >= (mu -lambda) * vmin)
        q = 1;
    end
elseif p == 0
    c = co;
elseif p > 1
    c = eta*c;
    [d_obs_n, p_obs_n, p] = dist_circle([x1,x2],r1,r2,num_obs,bo,c);
end

xplus = [x1; x2; p; q; c];
end