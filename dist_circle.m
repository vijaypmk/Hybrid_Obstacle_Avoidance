function [in_circle, obs_pos, obs_num] = dist_circle(x,r1,r2,num_obs,bo,control_space)
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


in_circle = 0;
obs_pos = 0;
obs_num = 0;

for i = 1:num_obs
    if sqrt((x(1) - r1(i))^2+(x(2) - r2(i))^2) <= (bo + control_space)
        in_circle = 1;
        obs_pos = i;
        obs_num = obs_num + 1;
    end
end
end