function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% Constants for the minimum altitude control task
uvms.min_dist = 1;
buffer = 0.5;
uvms.max_dist = uvms.min_dist + buffer;


% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;
uvms.v_rho = 0; 
uvms.altitude = 0; 


uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jv = []; 
uvms.Jv_a = []; 
uvms.Jv_v = []; 
%uvms.Jvlin = []; 
%uvms.Jvang = []; 
uvms.Jalt = []; 

uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
%uvms.xdot.v = [];
uvms.xdot.vlin = []; 
uvms.xdot.vang = []; 
uvms.xdot.min_alt = []; 
uvms.xdot.alt_land = []; 

uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
%uvms.A.v = zeros(6,6); 
uvms.A.vang = zeros(3,3); 
uvms.A.vlin = zeros(3,3); 
uvms.A.min_alt = zeros();
uvms.A.alt_land = zeros(); 


%uvms.Aa.v = eye(6); 
uvms.Aa.vlin = eye(3);
uvms.Aa.vang = eye(3);
uvms.Aa.ha = eye(1); 
uvms.Aa.t = zeros(6); 
uvms.Aa.alt_land = zeros(1); 

%uvms.A.v_rho = zeros(3,1); 

end

