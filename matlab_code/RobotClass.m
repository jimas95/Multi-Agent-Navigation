% 6/9/2018 not append to sample speeds zero current velocity. It stacks
% 11/9/2018 append also the current velocity at maximum speed

classdef RobotClass
% OBJ = A object  AGENT=B object   
    properties
        
        position     = zeros(2,1);
        velocity     = rand(2,1);   % current velocity
        velocity_ref = zeros(2,1);
        velocity_opt = zeros(2,1);
        target       = zeros(2,1);
        Cones
        priority
        Size=5;
        type                         % circle or square
        k_neighbors;
        Num_neighbors = 6;
        nearestNeighborLine
        max_speed
        speed_dif                         % maximum acceleration
        color
        agent_graphs
        line
        cone_triangle
        path
        directionLine
        acc_onOff
        
        
    end
        
    methods
        
        function obj=RobotClass(pos,tar,init_speed,color,alpha,max_speed, speed_dif, Size,type)
            %set agents position , velocity , target location 
            obj.position=pos;
            obj.target = tar;
            obj.velocity_ref = max_speed*(tar-pos)/norm(tar-pos);  % optimum velocity found
            obj.velocity_opt = max_speed*(tar-pos)/norm(tar-pos);  % optimum velocity to target
            obj.velocity=init_speed;                               % current velocity
            obj.priority = alpha;
            obj.max_speed = max_speed ; 
            obj.speed_dif = speed_dif ;
            obj.Size = Size;
            obj.type;
            obj.acc_onOff = 1 ;
            %creates the circle for plot
            x = Size*cos(linspace(0,2*pi));
            y = Size*sin(linspace(0,2*pi));
            obj.agent_graphs = hgtransform;
            obj.color=color;
            patch('XData',x,'YData',y,'FaceColor',color,'Parent',obj.agent_graphs)
            
            %initialize velocity,path lines for ploter
            obj.nearestNeighborLine = animatedline;
            obj.line = animatedline;
            obj.directionLine = animatedline('Color','black');
            obj.path = animatedline('Color',color);
        end
        
        function obj=update(obj,dt)
            obj.position = obj.position + obj.velocity_ref*dt; % moves agent to new position
            obj.agent_graphs.Matrix = makehgtform('translate',[obj.position; 0]); % gia ta grafika 
            
            if ~reachTarget(obj) % if agent reached his target --> stop 
                obj.velocity = obj.velocity_ref; % this is the velocity found me tin opoia den exoume colision
            else
                obj.velocity = [0;0];
                obj.priority = -2 ; 
            end
            
            plot(obj.target(1),obj.target(2),'o','Color',obj.color);
        end
        
        
        function Bool=reachTarget(obj) % returns true if distance of agent-target position is 
                                       % less than half agents size 
            distance = norm(obj.target-obj.position);
            if distance < obj.Size/2
                Bool = 1;
            else 
                Bool = 0;
            end
        end
        
        function drawNearestNeighborLine(obj,agents)
            clearpoints(obj.nearestNeighborLine);
            for i=1:1:obj.Num_neighbors
                addpoints(obj.nearestNeighborLine,obj.position(1),obj.position(2));
                addpoints(obj.nearestNeighborLine,agents(obj.k_neighbors(i)).position(1),agents(obj.k_neighbors(i)).position(2)); 
            end
        end
        
        function drawVelocity(obj) % draw agents velocity x4 ( 4 fores megaliteri gia na fenete ) 
            clearpoints(obj.directionLine);
            addpoints(obj.directionLine,obj.position(1),obj.position(2));
            addpoints(obj.directionLine,obj.position(1)+1*obj.velocity(1),obj.position(2)+1*obj.velocity(2));

        end
        
        function drawPath(obj) % zwgrafizei to path
            addpoints(obj.path,obj.position(1),obj.position(2));
        end
        
        function drawCone(obj) % draws all the cones that have been found ( without traslation of agentB )
           clearpoints(obj.line);
           if isempty(obj.Cones)
               if obj.priority >=0 
                   disp(['kati den paei kala me ta tangles points..!' ,num2str(obj.color)])
               end
           else
               for i=1:1:length(obj.Cones)
                   addpoints(obj.line,obj.Cones(i).Q1(1),obj.Cones(i).Q1(2));
                   addpoints(obj.line,obj.Cones(i).Q2(1),obj.Cones(i).Q2(2));
                   addpoints(obj.line,obj.Cones(i).Q3(1),obj.Cones(i).Q3(2));
                   addpoints(obj.line,obj.Cones(i).Q1(1),obj.Cones(i).Q1(2));
                end
           end
        end
        
        
        function best_vel=optimum_velocity_v2(obj)
           
            best_vel = [0;0;];                 %initialize the velocity variable we gona choose
            
            if obj.priority < 0
                best_vel=obj.velocity;
            elseif length(obj.Cones)==0
                best_vel = obj.max_speed*(obj.target-obj.position)/norm(obj.target-obj.position);
            else
                optimum_velocity = obj.max_speed*(obj.target-obj.position)/norm(obj.target-obj.position); %optimal velocity to reach target
                
                %create sample velocitys 
                N_points=50;
                if obj.acc_onOff==1     %use of acceleration
                [x y] = rand_circ(N_points,obj.velocity(1),obj.velocity(2),obj.speed_dif);
                for i=1:1:N_points
                    if norm(obj.velocity-[x(i); y(i)]) > obj.max_speed
                       x(i) = optimum_velocity(1);
                       y(i) = optimum_velocity(2);
                    end
                end
                else
                %without acceleration
                [x y] = rand_circ(N_points,0,0,obj.max_speed);
                end
                sample_velocity = [x' ;y'];
                sample_velocity = [sample_velocity optimum_velocity]; % adds an extra point at optimum velocity
                N_points = N_points + 1 ;
                
                if norm(obj.velocity) > 0.05*obj.max_speed     % 06/09/18 If current velocity is 0 then it stacks to it
                    sample_velocity = [sample_velocity obj.velocity obj.max_speed*obj.velocity/norm(obj.velocity)]; % adds two extra point at current velocity
                    N_points = N_points + 2 ;
                end
                
                N_cones=length(obj.Cones);
                % 3 main factors of choosing my new velocity
                %distance me optimum velocity 
                %distance of sample-current speed
               
                x_temp=repmat(optimum_velocity,1,N_points)-sample_velocity;
                distance_optimum_vel = sqrt(sum(x_temp.*x_temp,1));
                x_temp=repmat(obj.velocity,1,N_points)-sample_velocity;
                distance_current_vel = sqrt(sum(x_temp.*x_temp,1));
                
%               distance_optimum_vel = ones(N_cones,1)*vecnorm(repmat(optimum_velocity,1,N_points)-sample_velocity(:,:));  %vecnorm after 2017b
%               distance_current_vel = ones(N_cones,1)*vecnorm(obj.velocity-sample_velocity(:,:));
                
                collision_time = zeros(N_cones,N_points);
                
                for cone_num=1:1:N_cones
                    temp_Cone = obj.Cones(cone_num);
                    agentAB_vector = (temp_Cone.position-obj.position)*ones(1,N_points); %vector from agentA --> agentB
                    agentAB_distance = (norm(temp_Cone.position-obj.position)- obj.Size - temp_Cone.Size)*ones(1,N_points);
                    
                    speed_projection = dot(agentAB_vector,(sample_velocity-repmat(temp_Cone.speed,1,N_points)))/norm(agentAB_vector);
%                   speed_projection = dot(agentAB_vector,(sample_velocity-temp_Cone.speed))/norm(agentAB_vector);
%                     projection_v2 = ones(1,N_points)*(dot(agentAB_vector,temp_Cone.speed)/norm(agentAB_vector))
%                     speed = projection_v1; % needs fixing
                    collision_time(cone_num,:) = abs((agentAB_distance./speed_projection));  % take absolute value
%                      
                end
                
                bool_collision = zeros(length(obj.Cones),N_points);
                for cone_num=1:1:N_cones
                    temp_Cone = obj.Cones(cone_num);
                    for i=1:1:N_points
                        point = obj.position+sample_velocity(:,i);
                        bool_collision(cone_num,i)=temp_Cone.isIn(point);
                    end
                end
                
                
                w1 = 10000;
                w2 = 1.0;
                w3 = 0.5;
%                 temp=(bool_collision==1)*100000000;
%                 penalty =  (distance_optimum_vel+0.5*distance_current_vel).*(bool_collision==0)+temp;
                if N_cones >1
                    collision_penalty = max(([1]./collision_time).*(bool_collision));
                else            % if we have only one neighbour object
                    collision_penalty = ([1]./collision_time).*(bool_collision);
                end
                penalty = w1*collision_penalty + w2*distance_optimum_vel + w3*distance_current_vel;
                

                [~ ,index] =(min(penalty));   % XXXX
                best_vel=sample_velocity(:,index);
% For debugging add the next three lines               
%                 fprintf('index %d \t collision %d \t %5.3f \t %5.3f\n', index, bool_collision(index), best_vel(1), best_vel(2));
%                 fprintf('%d \t %5.2f \t %5.2f \t %5.2f \t %5.2f \t%7.2f\n', [bool_collision(1,:)' sample_velocity(1,:)' sample_velocity(2,:)' distance_optimum_vel' distance_current_vel' penalty']' )
%                 a=3;
            end
        end
        
        
        
        
        function obj=find_tang_points(obj,agent)

            P1 = obj.position;
            P2 = agent.position;
           
            r = (obj.Size+agent.Size); 
            
            % margin 10%  (does not allow object to get close than 10% r
            
            if r > 0.9*norm(P2-P1)
                r=0.9999*norm(P2-P1);
            end
            
            theta = asin(r/norm(P2-P1));  
            psi = atan2(P2(2)-P1(2),P2(1)-P1(1)) + [-1 1]*theta;
            L = cos(theta)*norm(P2-P1);
            Q = [P1 P1]+ L*[cos(psi);sin(psi)];
            
            if isreal(Q)
                if agent.priority < 0  % this is an obstacle -2 with speed 0,  -3 with standard speed
                    alpha = 1; 
                elseif agent.priority ==2    % this robot takes the priority of the other robot
                       alpha=obj.priority
                elseif obj.priority ==2    % this robot takes the priority of the other robot
                       alpha=1-agent.priority
                else
                    alpha = 0.5 + (obj.priority - agent.priority)/2.0;
                end
                cone = Cone(obj.position,Q(:,1),Q(:,2),agent.velocity, agent.position, agent.priority,agent.Size);
                
%              uncomment this line to run the original algorithm (1998) with no priorities
%                 cone = cone.traslate((agent.velocity));    % ORIGINAL method
                
%              uncomment this line to run the paper with priorities
                cone = cone.traslate((1-alpha)*obj.velocity + alpha*agent.velocity); % proposed method
                
% %              uncomment this line to run the paper with priorities equal to 0.5 i.e. v_new=(va+va')/2
%                 cone = cone.traslate((agent.velocity+obj.velocity)/2.0);

                obj.Cones = [ obj.Cones cone];
%                 bool = check_collision(obj,Q,agent);
%                 if bool==1
%                     disp('tha trakaroume!');
%                 end
             else  % we have allmost collision
                disp('complex num!');
            end
        end
    end
    

    
            
    
end