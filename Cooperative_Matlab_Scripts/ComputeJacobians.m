function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here

% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

%% Ex.0: Compute Jacobian for tool Position Control Task 

% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%% Ex 1.1: Compute Jacobian for the Vehicle Position Control Task
%Jacobian for the angular attitude wrt world frame
uvms.Jvang = [zeros(3,7) zeros(3,3) uvms.wTv(1:3, 1:3)];
%Jacobian for the linear velocity wrt world frame
uvms.Jvlin = [zeros(3,7) uvms.wTv(1:3, 1:3) zeros(3,3)]; 
uvms.Jv = [uvms.Jvlin; uvms.Jvang];

%% Compute Jacobian for the Horizontal Attitude Control Task 
%take into account the versor k of the vehicle frame and world frame 
w_kw = [0 0 1]';
v_kv = [0 0 1]'; 
%compute the projection of the k versor of the world frame on the vehicle
%frame
v_kw = (uvms.vTw(1:3,1:3)) * w_kw; 
% compute the misallignment of these two versor 
uvms.v_rho = ReducedVersorLemma(v_kw, v_kv); 

%direction of rho
if (norm(uvms.v_rho) > 0) 
    rho = uvms.v_rho/norm(uvms.v_rho);
else 
    rho = [0 0 0]'; 
end 

%The jacobian will be responsable only to drive this value to zero 
uvms.Jha = [zeros(1,7) zeros(1,3) rho']; 

%% Ex 1.2 : Compute Jacobian for the minimum altitude Control Task 
%%%% the distance to mantain is along z direction of the vehicle. then we
%%%% can compute the jacobian considering only the z axis, as follows: 
uvms.Jalt = [0 0 1 0 0 0] * uvms.Jv; 

% Jmatrix-->[1x13]
%% Ex 2.1: Compute Jacobian for altitude control task (Landing) 
% control task to regulate the altitude to zero
% Since the motion to control is on the z axis of the vehicle, we can
% implement the same Jacobian of the previous ex (1.2) 

%% Ex: Compute Jacobian for underactuated control task 
uvms.Jua = [zeros(6,7), eye(6)]; 

%% Ex 3:Compute Jacobian for Allignment x_vehicle/rock control task 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

%rock center coordinates projected on x-y plane wrt vehicle frame. 
v_rockcenter = [1 0 0 0; 0 1 0 0; 0 0 0 0] * uvms.vTw * [rock_center; 1]; 
%take the 
%v_iv = uvms.vTw(1:3,1:3) * [1 0 0]'; 
v_iv = [1 0 0]'; 
% compute the misallignment of these two versor 
uvms.v_xi = ReducedVersorLemma(v_rockcenter, v_iv); 

%direction of xi
if (norm(uvms.v_xi) > 0) 
    xi = uvms.v_xi/norm(uvms.v_xi);
else 
    xi = [0 0 0]'; 
end 

%The jacobian will be responsable only to drive this value to zero 
%[arms(1,7), ang(1,3), lin(1,3)] 
uvms.Jxi = [zeros(1,7) zeros(1,3) xi']; 

%% Ex 4.1:Compute Jacobian for Vehicle  Null  Velocities  Control  Task
uvms.Jnull = uvms.Jv;

%% Ex 4.2: Compute Jacobian for joint limit Control task 
uvms.J.jl = [eye(7), zeros(7,6)];  %We are directly acting on the joint

end