function [uvms] = ComputeActivationFunctions(uvms, mission)

%% Missions
    switch mission.phase
        case 1 %Safe Navigation Action 
            uvms.Aa.vc = eye(6); 
            uvms.Aa.min_alt = eye(1); % Safe Waypoint 
            uvms.Aa.jl = eye(1); %Safety
            uvms.Aa.ha = eye(1); % Safe Waypoint
            uvms.Aa.vang = eye(3); % Safe Waypoint
            uvms.Aa.vlin = eye(3); % Safe Waypoint
            uvms.Aa.null = zeros(6); 
            uvms.Aa.t = zeros(6); % Tool positioning 
            uvms.Aa.PreferredConfig = eye(4); %Optional

        case 2 %Floating Manipulation Action 
            uvms.Aa.jl = eye(1); 
            uvms.Aa.ha = eye(1); 
            uvms.Aa.min_alt = eye(1); 
            uvms.Aa.vang = zeros(3,3);    
            uvms.Aa.vlin = zeros(3,3);
            uvms.Aa.null = eye(6);
            uvms.Aa.t = eye(1); 
            uvms.Aa.PreferredConfig = eye(4);
            uvms.Aa.vc = eye(6);
    end 
    
        
%%%%%% Activation functions from previous exercises %%%%%%

%% Arm tool position control task 
uvms.A.t = eye(6) * uvms.Aa.t;

%% Vehicle position control task  
uvms.A.vang = eye(3) * uvms.Aa.vang; 
uvms.A.vlin = eye(3) * uvms.Aa.vlin; 

%% Horizontal attitude control task
uvms.A.ha = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha; 

%%  Minimum altitude control task
uvms.A.min_alt = DecreasingBellShapedFunction(uvms.min_dist, uvms.max_dist, 0, 1, uvms.altitude) * uvms.Aa.min_alt;

%% "Landing action" : activation function for altitude control task 
uvms.A.alt_land = eye(1) * uvms.Aa.alt_land;

%% Activation function for underactuated control task 
uvms.A.ua = diag([0 0 0 1 0 0]); 

%%  Allignment x_vehicle/rock Control task 
uvms.A.xi = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.v_xi)) * uvms.Aa.xi;

%% Vehicle Null Velocity control task 
error = norm(uvms.wTt(1:3,4) - uvms.wTg(1:3,4));
uvms.A.null =  DecreasingBellShapedFunction(0.01, 0.2, 0, 1, error) * uvms.Aa.null;

%% Joint limit control task 
for i = 1:length(uvms.q) 
    uvms.A.jl(i,i) = (DecreasingBellShapedFunction(uvms.jlmin(i), uvms.jlmin(i) + 0.1, 0, 1, uvms.q(i)) + IncreasingBellShapedFunction(uvms.jlmax(i) - 0.1, uvms.jlmax(i), 0, 1, uvms.q(i))) * uvms.Aa.jl;  
end 


%%%%%% New activation functions %%%%%%

%% Ex 5.1: Preferred configuration (first four joints)
diff = uvms.PreferredConfig - uvms.q(1:length(uvms.PreferredConfig));
for i = 1:length(uvms.PreferredConfig)
   uvms.A.PreferredConfig(i, i) = IncreasingBellShapedFunction(uvms.PreferredConfig(i) - 0.1, uvms.PreferredConfig(i),0,1,norm(diff(i)));
end
uvms.A.PreferredConfig = uvms.A.PreferredConfig * uvms.Aa.PreferredConfig;

%% Ex 6.1: Vehicle Constrained velocities
uvms.A.vc=eye(6);
