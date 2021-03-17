function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

%DECREASING
% This function is defined as follows
% ymax, if x < xmin
% ymin, if x > xmax
%INCREASING
% This function is defined as follows
% ymin, if x < xmin
% ymax, if x > xmax

%%uvms.Aa. _ = let the task be activated: if it is 1 the activation
%%function of the task will run. 

%% Missions
    switch mission.phase
        case 1 %Safe Navigation Action 
            uvms.Aa.min_alt = zeros(1); % Safe Waypoint 
            uvms.Aa.jl = eye(1); %Safety
            uvms.Aa.ha = eye(1); % Safe Waypoint
            uvms.Aa.vang = eye(3); % Safe Waypoint
            uvms.Aa.vlin = eye(3); % Safe Waypoint
            uvms.Aa.null = zeros(6); 
            uvms.Aa.t = zeros(6); 
            uvms.Aa.PreferredConfig = eye(4); %Optional
            uvms.Aa.vc = eye(6);
            
        case 2 %Floating Manipulation Action 
            uvms.Aa.jl = eye(1); %Safety 
            uvms.Aa.ha = eye(1); %Safety 
            uvms.Aa.min_alt = zeros(1); %Safety
            uvms.Aa.vang = zeros(3,3);    
            uvms.Aa.vlin = zeros(3,3);
            uvms.Aa.null = eye(6);
            uvms.Aa.t = eye(1); 
            uvms.Aa.PreferredConfig = eye(4); %Optional 
            uvms.Aa.vc = eye(6);

    end 
    
        

%% arm tool position control task 
uvms.A.t = eye(6) * uvms.Aa.t;

%% vehicle position control task 
%%%% Task 1.1: Activation function for vehicle position task 
%uvms.A.v = eye(6) * uvms.Aa.v;
%uvms.A.vang = eye(3);
%uvms.A.vlin = eye(3); 
uvms.A.vang = eye(3) * uvms.Aa.vang; 
uvms.A.vlin = eye(3) * uvms.Aa.vlin; 
%% horizontal attitude control task
%define a contraint st the z-axis of vehicle frame is always alligned with the z-axis of the world contraint
%we need to intruduce a bell shape function because it is an inequality
%task 

%if norm(rho) > 0.2, A = 1 
%if norm(rho) < 0.1, A = 0
%uvms.A.ha = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.v_rho));
uvms.A.ha = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha; 

%%  Minimum altitude control task
%if altitude < uvms.min_dist, A = 1 
%if altitude > uvms_min_dist + uvms.max_dist, A = 0 
uvms.A.min_alt = DecreasingBellShapedFunction(uvms.min_dist, uvms.max_dist, 0, 1, uvms.altitude) * uvms.Aa.min_alt;

%% "Landing action" : activation function for altitude control task 
% control task to regulate the altitude to zero: it is an equality task,
% then activation function is 1
uvms.A.alt_land = eye(1) * uvms.Aa.alt_land;


%% activation function for underactuated control task 
uvms.A.ua = diag([0 0 0 1 0 0]); 

%%  Allignment x_vehicle/rock Control task 
%inequality task 
uvms.A.xi = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.v_xi)) * uvms.Aa.xi;

%% Vehicle Null Velocity control task 
% non-reactive task 
%uvms.A.null = eye(6) * uvms.Aa.null; 

%
error = norm(uvms.wTt(1:3,4) - uvms.wTg(1:3,4));
uvms.A.null =  DecreasingBellShapedFunction(0.01, 0.2, 0, 1, error) * uvms.Aa.null;

%% Joint limit control task 
for i = 1:length(uvms.q) 
    uvms.A.jl(i,i) = (DecreasingBellShapedFunction(uvms.jlmin(i), uvms.jlmin(i) + 0.1, 0, 1, uvms.q(i)) + IncreasingBellShapedFunction(uvms.jlmax(i) - 0.1, uvms.jlmax(i), 0, 1, uvms.q(i))) * uvms.Aa.jl;  
end 

%% Ex 5.1: Preferred configuration (first four joints)
diff = uvms.PreferredConfig - uvms.q(1:length(uvms.PreferredConfig));
for i = 1:length(uvms.PreferredConfig)
   uvms.A.PreferredConfig(i, i) = IncreasingBellShapedFunction(uvms.PreferredConfig(i) - 0.1, uvms.PreferredConfig(i),0,1,norm(diff(i)));
end
uvms.A.PreferredConfig = uvms.A.PreferredConfig * uvms.Aa.PreferredConfig;

%% Ex 6.1
uvms.A.vc=eye(6);