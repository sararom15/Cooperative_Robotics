function [uvms, mission] = UpdateMissionPhase(uvms, mission)

    switch mission.phase        
        case 1 
            % Switch to floating manipulation action when the vehicle has
            % reached its goal position
            [w_ang, w_lin] = CartError(uvms.wTgv, uvms.wTv);
            
            if (norm(w_lin) < 0.1) && (norm(w_ang) < 0.1)
                mission.phase = 2; 
                uvms.time1 = mission.phase_time;
                mission.phase_time = 0;
            end 

    end 
                 
end

