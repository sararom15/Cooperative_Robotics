function [uvms] = ComputeJacobians(uvms)

%% J_tool
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];

% J_tool: arm and vehicle contributions
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];

% Juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%% J_vehicle_linear and J_vehicle_angular
uvms.Jvang = [zeros(3,7) zeros(3,3) uvms.wTv(1:3, 1:3)];
uvms.Jvlin = [zeros(3,7) uvms.wTv(1:3, 1:3) zeros(3,3)]; 

%% Compute Jacobian for the Horizontal Attitude 
% Versors k of the vehicle frame and world frame 
w_kw = [0 0 1]';
v_kv = [0 0 1]'; 
% k versor of the world frame projected on the vehicle frame
v_kw = (uvms.vTw(1:3,1:3)) * w_kw; 

% Misallignment of these two versors
uvms.v_rho = ReducedVersorLemma(v_kw, v_kv); 
if (norm(uvms.v_rho) > 0) 
    rho = uvms.v_rho/norm(uvms.v_rho);
else 
    rho = [0 0 0]'; 
end 

uvms.Jha = [zeros(1,7) zeros(1,3) rho']; 

%% Jacobian for the minimum altitude from the seafloor 
uvms.Jv = [uvms.Jvlin; uvms.Jvang]; 

uvms.Jalt = [0 0 1 0 0 0] * uvms.Jv; 

%% Jacobian for underactuated control task 
uvms.Jua = [zeros(6,7), eye(6)]; 

%% Jacobian for Vehicle Null velocities
uvms.Jnull = [uvms.Jvlin; uvms.Jvang]; 

%% Jacobian for Joint limits
uvms.J.jl = [eye(7), zeros(7,6)];  %We are directly acting on the joint

%% Ex 5.1: Jacobian for Preferred Configuration of the arm
uvms.JPreferredConfig = [eye(4), zeros(4,3), zeros(4,6)];

%% Ex 6.1: Jacobian for Vehicle Contrained velocity
uvms.Jvc=[zeros(6,7), eye(6)];

end