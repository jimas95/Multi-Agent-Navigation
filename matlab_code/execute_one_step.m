function [robots ,flag_stop]=execute_one_step(dt,robots,k_size)

        flag_stop = 0 ;
        ROBOTS_NUM = length(robots);
        for i=1:1:ROBOTS_NUM % finds tangled points of agentA only for those who are closed to him
            agentA = robots(i).position;
            agentA_Size = robots(i).Size;
          if robots(i).priority >= 0          % do not consider obstacles or agents reached their position (increase speed)
            for j=1:1:ROBOTS_NUM
                agentB = robots(j).position;
                agentB_size = robots(j).Size;
                dist = norm(agentA-agentB)-agentB_size;  % cheks the closest point of near objects
                if dist < agentA_Size*k_size && i~=j
                     robots(i) = robots(i).find_tang_points(robots(j));
                end
            end
          end
        end

        
        for i=1:1:ROBOTS_NUM
            robots(i).velocity_ref = robots(i).optimum_velocity_v2();
        end
        
        %this is for running parrale programing , causes problem with graphics!
%         parpool('local',2)       %  use max 4 processors in parallel 
%         parfor i=1:ROBOTS_NUM         %  use parfor like a normal for loop
%         robots(i).velocity_ref = robots(i).optimum_velocity(); %  do something, e.g.,  call a function for each element i 
%         end
%         delete(gcp('nocreate'))       %  close parallel processes 
    
        for i=1:1:ROBOTS_NUM
%             if i==1
%               robots(i).drawCone();
%               robots(i).drawNearestNeighborLine(robots);
%             end

%             robots(i).drawVelocity(); % draws direction of velocity
%             robots(i).drawPath();          % drasw agents path
            robots(i).Cones = [];          %clear list of cones

        end
        
        for i =1:1:ROBOTS_NUM
            robots(i) = robots(i).update(dt); % updates staff for agent
            flag_stop=flag_stop + robots(i).priority; 
        end


    
end