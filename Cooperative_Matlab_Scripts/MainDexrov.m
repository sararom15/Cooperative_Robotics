function MainDexrov
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 60;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('DexROV');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';

% initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.3;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% Vehicle goal position 
%uvms.v_goalPosition=[pipe_center(1), pipe_center(2)-1, pipe_center(3)+1.35]';

% the goal has defined trying the simulation without null vehicle
% velocities task and taking the final configuration position of the
% vehicle when the tool has reached its goal position. 

uvms.v_goalPosition = [-2.3, 10.1, -29.8]'; 
uvms.wRgv = rotation(0,0.06,-pi/2);
uvms.wTgv = [uvms.wRgv uvms.v_goalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);
tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
   
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    
    %% TPIK1
    rhop = zeros(13,1);
    Qp = eye(13); 

    %% Joint limit control task (safety task)
    [Qp, rhop] = iCAT_task(uvms.A.jl, uvms.J.jl, Qp, rhop, uvms.xdot.jl, 0.0001, 0.01, 10);

    %% Minimum altitude control task (safety task)
    [Qp, rhop] = iCAT_task(uvms.A.min_alt, uvms.Jalt, Qp, rhop, uvms.xdot.min_alt, 0.0001, 0.01, 10);

    %% Horizontal Attitude control task (safety task)
    [Qp, rhop] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, rhop, uvms.xdot.ha, 0.0001, 0.01, 10);

    %% "Safe Navigation action": Vehicle Position control task
    [Qp, rhop] = iCAT_task(uvms.A.vang, uvms.Jvang, Qp, rhop, uvms.xdot.vang, 0.0001, 0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.vlin, uvms.Jvlin, Qp, rhop, uvms.xdot.vlin, 0.0001, 0.01, 10);

    %% Vehicle Null velocity non-reactive control task
    [Qp, rhop] = iCAT_task(uvms.A.null, uvms.Jnull, Qp, rhop, uvms.xdot.null, 0.0001, 0.01, 10);

    %% Tool position control task
    [Qp, rhop] = iCAT_task(uvms.A.t, uvms.Jt, Qp, rhop, uvms.xdot.t, 0.0001, 0.01, 10);
    
    %% Preferred configuration (first four joints)
    [Qp, rhop] = iCAT_task(uvms.A.PreferredConfig,    uvms.JPreferredConfig,    Qp, rhop, uvms.xdot.PreferredConfig,  0.0001,   0.01, 10);
    %%
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    
    %% TPIK2
    rhop2 = zeros(13,1);
    Qp2 = eye(13); 
    
    %% Vehicle constrained velocity (non-reactive control task) 
    [Qp2, rhop2] = iCAT_task(uvms.A.vc, uvms.Jvc, Qp2, rhop2, uvms.xdot.vc, 0.0001, 0.01, 10);


    %% Joint limit control task (safety task)
    [Qp2, rhop2] = iCAT_task(uvms.A.jl, uvms.J.jl, Qp2, rhop2, uvms.xdot.jl, 0.0001, 0.01, 10);

    %% Minimum altitude control task (safety task)
    [Qp2, rhop2] = iCAT_task(uvms.A.min_alt, uvms.Jalt, Qp2, rhop2, uvms.xdot.min_alt, 0.0001, 0.01, 10);

    %% Horizontal Attitude control task (safety task)
    [Qp2, rhop2] = iCAT_task(uvms.A.ha, uvms.Jha, Qp2, rhop2, uvms.xdot.ha, 0.0001, 0.01, 10);

    %% "Safe Navigation action": Vehicle Position control task
    [Qp2, rhop2] = iCAT_task(uvms.A.vang, uvms.Jvang, Qp2, rhop2, uvms.xdot.vang, 0.0001, 0.01, 10);
    [Qp2, rhop2] = iCAT_task(uvms.A.vlin, uvms.Jvlin, Qp2, rhop2, uvms.xdot.vlin, 0.0001, 0.01, 10);

    %% Vehicle Null velocity non-reactive control task
    [Qp2, rhop2] = iCAT_task(uvms.A.null, uvms.Jnull, Qp2, rhop2, uvms.xdot.null, 0.0001, 0.01, 10);

    %% Tool position control task
    [Qp2, rhop2] = iCAT_task(uvms.A.t, uvms.Jt, Qp2, rhop2, uvms.xdot.t, 0.0001, 0.01, 10);
    
    %% Preferred configuration (first four joints)
    [Qp2, rhop2] = iCAT_task(uvms.A.PreferredConfig,    uvms.JPreferredConfig,    Qp2, rhop2, uvms.xdot.PreferredConfig,  0.0001,   0.01, 10);
    %%
    [Qp2, rhop2] = iCAT_task(eye(13),     eye(13),    Qp2, rhop2, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    
   
    % get the two variables for integration
    uvms.q_dot = rhop2(1:7);
    uvms.p_dot = rhop(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    
    %disturbance in x and y axis of the vehicle 
    uvms.disturb = zeros(1,3); 

    for i = 1:2 
        uvms.disturb(i) = 0.05*sin(2*0.1*pi*t);
    end 
    
    uvms.p_dot(1:3) = uvms.p_dot(1:3) + (uvms.vTw(1:3,1:3) * uvms.disturb'); 
    
    
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        %uvms.p'
        uvms.q(1:4)
        mission.phase
        uvms.p

        
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end 