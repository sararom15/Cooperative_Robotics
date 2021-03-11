function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
    
        %two actions, therefore define 1 case which allow the switch
        %between one action and the other 
        
        case 1 
            %switch to floating manipulation action when the vehicle has
            %reached its goal position
            [w_ang, w_lin] = CartError(uvms.wTgv, uvms.wTv);
            if (norm(w_lin) < 0.1) && (norm(w_ang) < 0.1)
                %switch action
                mission.phase = 2; 
                uvms.time1 = mission.phase_time;
                %update time 
                mission.phase_time = 0;
            end 

    end 
                 
end

