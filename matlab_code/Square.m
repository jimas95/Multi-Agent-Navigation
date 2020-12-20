% 11/09/2018 Add get_cones_V3
classdef Square
    
   properties
       position
       velocity
       color
       size
       graphs
       line
   end  
   
   methods
      % "size" is the length of the diagonal/2  (aktina perigegramenou kyklou)
      % obj is the square
       function obj=Square(pos,speed,color,size)
          obj.position = pos;
          obj.velocity = speed;
          obj.color = color;
          obj.size =  sqrt(2)*size/2.0;   % size of a square side/2
          x_pos = obj.position(1)+[1 -1  -1  1]*obj.size;
          y_pos = obj.position(2)+[1  1  -1 -1]*obj.size;
          obj.graphs = hgtransform;
          patch('XData',x_pos,'YData',y_pos,'FaceColor',color,'Parent',obj.graphs)
%           obj.line = animatedline('Color',obj.color);
       end
       
        function obj=update(obj,dt)
%            clearpoints(obj.line);
%            pos = obj.position + [1 -1  -1  1 1; 
%                                  1  1  -1 -1 1]*obj.size ;
%            for i=1:1:4
%                addpoints(obj.line,pos(:,i),pos(:,i+1));
%            end
           
           obj.position = obj.position + obj.velocity*dt; % moves agent to new position
           obj.graphs.Matrix = makehgtform('translate',[obj.position; 0]); % gia ta grafika 
            
        end
        % the speed of the square is set to 0
		function [cone, flag] = get_cones_V3(obj,position,R_agent,agent_vel)
%                 Size = 0     ;   
            size = obj.size +R_agent; % (a/2)+agent_size
 % square is an obstacle. Its priority is set to 0 
            alpha = 0.0;  % obj priority (change it in the next version)
            cones = [];
            pos = repmat(obj.position,1,4) + [1 -1  -1  1 ;
                                  1  1  -1 -1]*size ;  %minkoski sum of square and circle
            flag = 0 ;
            pos_x=position(1); pos_y=position(2);  % agent's position
            S1=pos(:,1); S2=pos(:,2); S3=pos(:,3); S4=pos(:,4); % 4 square corners
            if pos(2,1)<pos_y % panw apo to tetragono
                
                if pos(1,1) < pos_x % panw-dexia apo to tetragono
                    Q1 = S2;
                    Q2 = S4;
                    plasmatiko_size = obj.size +R_agent;
                elseif pos(1,2) > pos_x %panw-aristera 
                    Q1 = S1;
                    Q2 = S3;
                    plasmatiko_size = obj.size +R_agent;

                else %anamesa-panw
                    Q1 = S1;
                    Q2 = S2;
                    plasmatiko_size = obj.size/sqrt(2) + R_agent;
                end
                
            elseif pos(2,4) > pos_y % katw apo to tetragono
                
                if pos(1,4) < pos_x %katw dexia
                    Q1 = S1;
                    Q2 = S3;
                    plasmatiko_size = obj.size + R_agent;
                elseif pos(1,3) > pos_x % katw aristera
                    Q1 = S2;
                    Q2 = S4;
                    plasmatiko_size = obj.size + R_agent;
                else                         % katw kai anamesa
                    Q1 = S3;
                    Q2 = S4;
                    plasmatiko_size = obj.size/sqrt(2) + R_agent;
                end
                
                
            else %dexia h aristeara h mesa sto tetragono
                
                if pos(1,3) > pos_x % anamesa aristera
                    Q1 = S3;
                    Q2 = S2;
                    plasmatiko_size = obj.size/sqrt(2) + R_agent;
                elseif pos(1,4) < pos_x % anamesa dexia
                    Q1 = S1;
                    Q2 = S4;
                    plasmatiko_size = obj.size/sqrt(2) + R_agent;
                else                        % mesa !
                    disp('collision!');
                    flag = 1;
                end
               
            end
            
            if flag==0 %  && Q1(1,1)~=0   
                
            correct_margin=0;
            margin_sqr=1.2;  % it will check if agent is less than x% of the objects size
            margin_cone=0.05; % then it will move cone y% further out
            pos_m = repmat(obj.position,1,4) + [1 -1  -1  1 ;
                                  1  1  -1 -1]*size*margin_sqr ;
            if (pos_x-pos_m(1,1))*(pos_x-pos_m(1,2))<0 & (pos_y-pos_m(2,1))*(pos_y-pos_m(2,3))<0
                correct_margin=margin_cone*size*(position-mean(pos')')/norm((position-mean(pos')')); % move away on the line that connects centers
            end
            
            cone = Cone(position,Q1,Q2,obj.velocity, obj.position, 1-alpha, plasmatiko_size);     
            cone = cone.traslate((1-alpha)*obj.velocity + alpha*agent_vel+correct_margin); % proposed method
            cones = [cones cone]; 
                    
            else
                cone = [];
            end
            
            
        end
        
		function cone = get_cones_V2(obj,position,Size,speed)
%                 Size = 0     ;   
            size = obj.size +Size; % sqrt(2)*(a/2)+agent_size
 
            alpha = 0.0;
            cones = [];
            pos = repmat(obj.position,1,4) + [1 -1  -1  1 ;
                                  1  1  -1 -1]*size ;
            flag = 0 ;
            if pos(2,1)<position(2) % panw apo to tetragono
                
                if pos(1,1) < position(1) % panw-dexia apo to tetragono
                    Q1 = pos(:,2);
                    Q2 = pos(:,4);
                    plasmatiko_size = obj.size +Size;
                elseif pos(1,2) > position(1) %panw-aristera 
                    Q1 = pos(:,1);
                    Q2 = pos(:,3);
                    plasmatiko_size = obj.size +Size;

                else %anamesa-panw
                    Q1 = pos(:,1);
                    Q2 = pos(:,2);
                    plasmatiko_size = obj.size/sqrt(2) + Size;
                end
                
            elseif pos(2,4) > position(2) % katw apo to tetragono
                
                if pos(1,4) < position(1) %katw dexia
                    Q1 = pos(:,1);
                    Q2 = pos(:,3);
                    plasmatiko_size = obj.size + Size;
                elseif pos(1,3) > position(1) % katw aristera
                    Q1 = pos(:,2);
                    Q2 = pos(:,4);
                    plasmatiko_size = obj.size + Size;
                else                         % katw kai anamesa
                    Q1 = pos(:,3);
                    Q2 = pos(:,4);
                    plasmatiko_size = obj.size/sqrt(2) + Size;
                end
                
                
            else %dexia h aristeara h mesa sto tetragono
                
                if pos(1,3) > position(1) % anamesa aristera
                    Q1 = pos(:,3);
                    Q2 = pos(:,2);
                    plasmatiko_size = obj.size/sqrt(2) + Size;
                elseif pos(1,4) < position(1) % anamesa dexia
                    Q1 = pos(:,1);
                    Q2 = pos(:,4);
                    plasmatiko_size = obj.size/sqrt(2) + Size;
                else                        % mesa !
                    disp('collision!');
                    flag = 1;
                end
               
            end
            
            if flag==0 && Q1(1,1)~=0  % 
            cone = Cone(position,Q1,Q2,obj.velocity, obj.position, 0.99,plasmatiko_size);
            cone = cone.traslate((1-alpha)*obj.velocity + alpha*speed); % proposed method
            cones = [cones cone];
            else
                cone = [];
            end
            
            
        end
       
        function [cone, flag]=get_cones(obj,position,Size,speed)
            flag=0 ; % counts collisions
            size = obj.size +Size; % sqrt(2)*a/2
            alpha = 0.01;
            cones = [];
            pos = repmat(obj.position,1,4) + [1 -1  -1  1 ; 
                                  1  1  -1 -1]*size ;
                              
            max_angle = 0;       
            for i=1:1:4
                for j=i+1:1:4
                    vec1 = pos(:,i) - position;
                    vec2 = pos(:,j) - position;
                    dot_product = dot(vec1,vec2);
                    angle = acosd(dot_product/(norm(vec1)*norm(vec2)));
                    if angle > max_angle
                        pos1=vec1; pos2=vec2;
                        max_angle = angle;
                        Q = [i;j];
                    end
                end
            end
%   uncomment next 2 lines for debugging           
%             fprintf('angle %4.1f \n vec1 %d  (%d, %d) \n vec2 %d (%d,%d)\n', max_angle, norm(pos1), pos1(1), pos(2), norm(pos2), pos2(1), pos2(2));
%             fprintf('XX %d yy %d \n',(position(1)-(pos(1,1)))*(position(1)-(pos(1,2))),(position(2)-(pos(2,1)))*(position(2)-(pos(2,3))));
            
            %            max_angle
            cone = Cone(position,pos(:,Q(1)),pos(:,Q(2)),obj.velocity, obj.position, 0.99, obj.size);
            % margin for square
            
%             if max_angle > 160   % use a parametric equation of a line
%                 t1=2;  t2=-2; 
%                 x1=pos(1,Q(1)); x2=pos(1,Q(2));y1=pos(2,Q(1)); y2=pos(2,Q(2));
%                 y1_new=y1+(y2-y1)*t1; x1_new=x1+(x2-x1)*t1;
%                 y2_new=y1+(y2-y1)*t2; x2_new=x1+(x2-x1)*t2;
%                 cone = Cone(position,[x1_new y1_new]' ,[x2_new y2_new]',obj.velocity, obj.position, 0.99, obj.size);
%             end
%             % do not let it come too close in the corner
            correct_margin=0;
            margin_sqr=1.4;
            margin_cone=0.05;
            if (position(1)-margin_sqr*(pos(1,1)))*(position(1)-margin_sqr*(pos(1,2)))<0 & (position(2)-margin_sqr*(pos(2,1)))*(position(2)-margin_sqr*(pos(2,3)))<0
                correct_margin=margin_cone*size*(position-mean(pos')')/norm((position-mean(pos')'));
                if (position(1)-(pos(1,1)))*(position(1)-(pos(1,2)))<0 & (position(2)-(pos(2,1)))*(position(2)-(pos(2,3)))<0
                    disp('collision')
                    flag=1;
                end
            end
            
                 
            cone = cone.traslate((1-alpha)*obj.velocity + alpha*speed+correct_margin); % proposed method
            cones = [cones cone];
            
%             cone = Cone(position,pos(:,2),pos(:,4),obj.velocity, obj.position, 1.0,sqrt(2)*obj.size);
%             cone = cone.traslate((1-alpha)*obj.velocity + alpha*speed); % proposed method
%             cones = [cones cone];
% 
%                         cone = Cone(position,pos(:,1),pos(:,3),obj.velocity, obj.position, 1.0,sqrt(2)*obj.size);
%             cone = cone.traslate((1-alpha)*obj.velocity + alpha*speed); % proposed method
%             cones = [cones cone];

       end
       
       
   end
    
end