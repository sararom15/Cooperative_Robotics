function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
%%%%%%%%%%%%% [angular velocities; linear velocities]%%%%%%%%%%%%%%%%%%%%
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
%angular velocity sono le stesse del end-effector perchè il tool è
%attaccato al end effector e ruotano insieme. invece le velocità lineari
%sono 

uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
%jACOBIAN OF THE ARM wrt tool + jacobiand of the VEHICLE wrt tool
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%% Ex 1.1: Compute Jacobian for the Vehicle Position Control Task
%%%% Task 1.1: Compute Jacobian for the Vehicle Position Control Task
%Jacobian interested in only the vehicle motion -> 
%Frist 7 columns (related to arms) are 0
uvms.Jv_a = [zeros(6,7)]; 
%the linear and angular velocity are the velocity of the vehicle (given wrt
%vehicle frame, projected on <w> -> use the rotation matrix 
uvms.Jv_v = [uvms.wTv(1:3, 1:3) zeros(3,3); zeros(3,3) uvms.wTv(1:3, 1:3)]; 
uvms.Jv = [uvms.Jv_a    uvms.Jv_v]; 

uvms.Jvang = [zeros(3,7) zeros(3,3) uvms.wTv(1:3, 1:3)];
uvms.Jvlin = [zeros(3,7) uvms.wTv(1:3, 1:3) zeros(3,3)]; 

%% Compute Jacobian for the Horizontal Attitude 
%take into account the versor k of the vehicle frame and world frame 
w_kw = [0 0 1]';
v_kv = [0 0 1]'; 
%compute the projection of the k versor of the world frame on the vehicle
%frame
v_kw = (uvms.vTw(1:3,1:3)) * w_kw; 
% compute the misallignment of these two versor 
uvms.v_rho = ReducedVersorLemma(v_kw, v_kv); 
% avoid division by 0

%direction of rho
if (norm(uvms.v_rho) > 0) 
    
    rho = uvms.v_rho/norm(uvms.v_rho);
else 
    rho = [0 0 0]'; 
end 

%The jacobian will be responsable only to drive this value to zero 
uvms.Jha = [zeros(1,7) zeros(1,3) rho']; 

%% Ex 1.2 : Jacobia for the minimum altitude from the seafloor 
%%%% the distance to mantain is along z direction of the vehicle. then we
%%%% can compute the jacobian considering only the  as follow: 
uvms.Jalt = [0 0 1 0 0 0] * uvms.Jv; 

% Jmatrix-->[1x13]
%% Ex 2.1: "Landing action" : reference for altitude control task 
% control task to regulate the altitude to zero
% Since the motion to control is on the z axis of the vehicle, we can
% implement the same Jacobian of the previous ex (1.2) 

end