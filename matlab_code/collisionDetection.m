% 10/09/2018 Checks only the unchecked  (j=i+1:length(agents) )

function collision= collisionDetection(agents)

    collision = 0 ;% --> false did not detect colision

for i=1:length(agents)

    for j=i+1:length(agents)
        vector = agents(i).position - agents(j).position;
        distance =  norm(vector);
        if distance < agents(i).Size+agents(j).Size
            disp(['Collision detected! agents:', num2str(i), ',' , num2str(j)]);
            collision = 1 ;
        end
    end
end
end

function collision = collisionDetection_v0(agents)

    collision = 0 ;% --> false did not detect colision

for i=1:length(agents)

    for j=1:length(agents)
        vector = agents(i).position - agents(j).position;
        distance =  norm(vector);
        if distance < agents(i).Size+agents(j).Size && i~=j
            disp(['Collision detected! agents:', num2str(i), ',' , num2str(j)]);
            collision = 1 ;
        end
    end
end
end