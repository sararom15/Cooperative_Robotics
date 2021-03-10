function [uvms, mission] = UpdateMissionPhase(uvms, mission)
%     switch mission.phase
%         %two actions, therefore define 1 case which allow the switch
%         %between one action and the other 
%         
%         case 1 
%             %switch to allignment action when the goal for the vehicle is
%             %achieved
%             [w_ang, w_lin] = CartError(uvms.wTg, uvms.wTv);
%             if (norm(w_lin) < 0.1) && (norm(w_ang) < 0.1)
%                 %switch action
%                 mission.phase = 2; 
%                 uvms.time1 = mission.phase_time;
%                 %update time 
%                 mission.phase_time = 0; 
%                 
%                 
%             end 
%         case 2  
% %             %switch to landing action when the vehicle is alligned with the
% %             %rock 
% %             if (norm(uvms.v_xi) < 0.1)
% %                 mission.phase = 3; 
% %                 uvms.time2 = mission.phase_time; 
% %                 mission.phase_time = 0; 
% %             end 
% %         case 3 
% %            if(uvms.sensorDistance < 0.11)
% %                mission.phase = 4;
% %                uvms.time3 = mission.phase_time; 
% %                mission.phase_time = 0;
% %            end
% %            
% %         case 4
% %             
%      end 
%                 
end

