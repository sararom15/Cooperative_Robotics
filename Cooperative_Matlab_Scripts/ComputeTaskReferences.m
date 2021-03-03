function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

%% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);


%% Ex 1.1: reference for vehicle position control task 
%%%%  Compute the reference for vehicle position control task 
% Compute the Cartesian error between the goal frame and the vehicle frame with respect to the world frame
[w_ang, w_lin] = CartError(uvms.wTgv, uvms.wTv);

%attitude vehicle control task 
uvms.xdot.vang = Saturate(0.2 * w_ang, 0.2); 

%position vehicle control task 
uvms.xdot.vlin = Saturate(0.2 * w_lin, 0.2);

%uvms.xdot.v= 0.2*[w_lin; w_ang]; 
%uvms.xdot.v(1:3)=Saturate(uvms.xdot.v(1:3),0.2);
%uvms.xdot.v(4:6)=Saturate(uvms.xdot.v(4:6),0.2);

%% reference for horizontal attitude
uvms.xdot.ha = 0.2*(0-norm(uvms.v_rho));

%% Ex 1.2 : reference for minimum altitude 
%%%% Compute the reference for the safety minimun altitude control task
%%%% (velocity along the z direction) 
%Compute the altitude according to the sensor measurement along k versor 
uvms.altitude = [0 0 1] * uvms.wTv(1:3, 1:3) * [0; 0; uvms.sensorDistance]; 
%Compute the minimun reference rate for the control task 
%since we define the minimum distance to mantain with the seafloor = 0.5 

%Compute the minimum altitude as the difference between the threshold and
%the distance measure from the sensor, moltiply by a gain (coefficient velocity = 0.5, in such a
%way it moves faster) 
uvms.xdot.min_alt = 0.5*(uvms.max_dist - uvms.altitude); 
%Saturate it 
uvms.xdot.min_alt = Saturate(uvms.xdot.min_alt, 0.5); 

%% Ex 2.1: "Landing action" : reference for altitude control task 
% control task to regulate the altitude to zero 
%the altitude (/uvms.altitude) must tend to zero slowly 
coeff_velocity = 0.2; 
uvms.xdot.alt_land = coeff_velocity * (0.1 - uvms.altitude);
uvms.xdot.alt_land = Saturate(uvms.xdot.alt_land, 0.2); 

