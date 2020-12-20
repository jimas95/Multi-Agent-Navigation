classdef Cone
   
    properties
        Q1
        Q2
        Q3
        speed
        position
        a
        Size
        
    end
    
    methods
        
        
        function obj=Cone(Q1,Q2,Q3,speed,position,alpha,Size)
            obj.Q1 = Q1;
            obj.Q2 = Q2;
            obj.Q3 = Q3;
            obj.speed = speed;
            obj.position = position;
            obj.a = alpha ;    % it is not used?
            obj.Size = Size;
        end
        
        function Bool=isIn(obj,point)
            
            vector1 = obj.Q2 - obj.Q1; %vector AB
            vector2 = obj.Q3 - obj.Q1; %vector AC
            vector_point = point - obj.Q1; %vector AP
            dot_product = (vector1/norm(vector1) + vector2/norm(vector2))'*vector_point;
            cross_product1 = vector1(1)*vector_point(2) - vector_point(1)*vector1(2);
            cross_product2 = vector2(1)*vector_point(2) - vector_point(1)*vector2(2);

            if cross_product1*cross_product2<0 && dot_product > 0 
                Bool = 1 ;     % 1 means collision
            else
                Bool = 0 ;
            end
        end
        
        function obj=traslate(obj,trasl)
            obj.Q1 = obj.Q1+trasl;
            obj.Q2 = obj.Q2+trasl;
            obj.Q3 = obj.Q3+trasl;            
        end
        
    end
    
    
end