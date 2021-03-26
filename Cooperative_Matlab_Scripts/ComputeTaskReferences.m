function [uvms] = ComputeTaskReferences(uvms, mission)

%% Reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];

uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.5);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.5);

%% Reference for vehicle position control task 
[w_ang, w_lin] = CartError(uvms.wTgv, uvms.wTv);

uvms.xdot.vang = Saturate(0.8 * w_ang, 0.3); 
uvms.xdot.vlin = Saturate(0.8 * w_lin, 0.3);

%% Reference for horizontal attitude
uvms.xdot.ha = 0.2*(0-norm(uvms.v_rho));

%% Reference for minimum altitude 
uvms.altitude = [0 0 1] * uvms.wTv(1:3, 1:3) * [0; 0; uvms.sensorDistance]; 
uvms.xdot.min_alt = 0.5*(uvms.max_dist - uvms.altitude); 

uvms.xdot.min_alt = Saturate(uvms.xdot.min_alt, 0.5); 

%% Reference rate for underactuated control task 
uvms.xdot.ua = uvms.p_dot; 

%% Reference for vehicle null velocity control task 
uvms.xdot.null = zeros(6,1);

%% Reference for joint limit control task 
for i = 1:length(uvms.q) 
    uvms.mean(i) = (uvms.jlmin(i) + uvms.jlmax(i))/2; 
    uvms.xdot.jl(i,1)= 0.2*Saturate(uvms.mean(i)-uvms.q(i), 0.2);
    
end 

%% Ex 5.1 : Reference for Preferred Configuration for the Joints 
uvms.xdot.PreferredConfig = - 0.1 * (uvms.q(1:length(uvms.PreferredConfig)) - uvms.PreferredConfig);

%% Ex 6.1: Reference for Vehicle Constrained velocity
uvms.xdot.vc = uvms.p_dot; 



