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

%% Mission 
    switch mission.phase
        case 1 %Safe Navigation Action 
            uvms.Aa.min_alt = eye(1); 
            uvms.Aa.ha = eye(1); 
            %uvms.Aa.v = eye(6);
            
            uvms.Aa.vang = eye(3); 
            uvms.Aa.vlin = eye(3); 
            
            uvms.Aa.alt_land = zeros(1); 
            uvms.Aa.xi = zeros(1);
            
            uvms.Aa.t = zeros(6); 
            
        case 2 
            uvms.Aa.min_alt = eye(1); 
            uvms.Aa.ha = eye(1); 
            %uvms.Aa.v = eye(6);
            uvms.Aa.vang = zeros(3); 
            % we want to mantain the goal position 
            uvms.Aa.vlin =  eye(3);
            uvms.Aa.xi = eye(1); 
            uvms.Aa.alt_land = zeros(1); 
            
            %uvms.Aa.t = zeros(6); 

        case 3 %Landing Action
            uvms.Aa.ha = eye(1);
            uvms.Aa.min_alt = zeros(1); 
            
            %uvms.Aa.xi = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            %mantain the position during the landing 
            uvms.Aa.xi = eye(1); 
            
            %if the request is to land mantaining the attitude of the vehicle, then
            %the uvms.Aa.vang should be set as follows: 
            %uvms.Aa.vang = eye(3);    
            %otherwise deactivate the vehicle position control task and the vehicle attitude control task  as follows: 
            uvms.Aa.vang = zeros(3,3);             
            uvms.Aa.vlin = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            
            %activate the altitude control task 
            uvms.Aa.alt_land = IncreasingBellShapedFunction(0, 0.05, 0, 1, mission.phase_time) * eye(1); 
            %uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(6); 
    end 
        

%% arm tool position control task 
uvms.A.t = eye(6) * uvms.Aa.t;

%% Ex 1.1: vehicle position control task 
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

%% Ex 1.2: Minimum altitude control task
%if altitude < uvms.min_dist, A = 1 
%if altitude > uvms_min_dist + uvms.max_dist, A = 0 
uvms.A.min_alt = DecreasingBellShapedFunction(uvms.min_dist, uvms.max_dist, 0, 1, uvms.altitude) * uvms.Aa.min_alt;

%% Ex 2.1: "Landing action" : activation function for altitude control task 
% control task to regulate the altitude to zero: it is an equality task,
% then activation function is 1
uvms.A.alt_land = eye(1) * uvms.Aa.alt_land;


%% Ex : activation function for underactuated control task 
uvms.A.ua = diag([0 0 0 1 0 0]); 

%% Ex 3: Jacobian Allignment x_vehicle/rock 
%inequality task 
uvms.A.xi = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.v_xi)) * uvms.Aa.xi;
