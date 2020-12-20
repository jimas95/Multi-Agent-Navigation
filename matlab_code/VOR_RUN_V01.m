% profile on
clear all;
close all;

% V01 06/09/2018 
%   add example 5 and example 6
%   Print steps and total time in the final plot
%   add relative matlab speed for normalizing execution time
% 
comp_type=[computer, '...'] ; % in order to have at least 6 characters
if comp_type(1:6) == 'PCWIN6'
    disp('Computer is VAIO PCWIN64')
    rel_speed_max=0.1654;
    rel_speed_aver=0.7366;
elseif comp_type(1:6) == 'MACI64'
    disp('Computer is MACPro MACI64')
    rel_speed_max=0.1794;
    rel_speed_aver=0.5955;
else
    disp('No computer match found')
    rel_speed_max=0.5;
    rel_speed_aver=0.1;
end

%EXAMPLES   
% EX=1:  2 robots You can choose different alphas to see their behavor
% EX=2:  4 times nmax robots placed in a circle. 
% EX=3:  4 Groups 0f 9 of robots moving through obstacles
% EX=4:  Robots in a raw with a moving obstacle
% EX=5:  Robots trapped in a hole
% EX=6:  Robots with square obstacles
% EX=7:  Robots in a raw with a moving square obstacle
EX=2;
%INIT PARAMETERS
%SELECT UPDATE MODE  mod=1  batch mode  mod=2 step mode
mod=1 ; % BATCH MODE

% The display area
Area_X = 150; Area_Y= 100;
%dt the time step. If collisions reduce it
dt = 0.05;  % the update time Every dt we calcuate the speeds again

% The knowledge area. It determines how far the robot sees. 
% The knowledge distance is k_size times its size
k_size=10 ; %checks only neighbors been in a distance k_times its size

% The robots list
robots = [] ;

% RobotClass(current_pos,target_pos,init_speed,color,alpha,max_speed, acc, Size, type)  


if EX==1
    % TWO or THREE ROBOTS
    dt=0.1; k_size=30;
    alpha1=0.0 ;  v_max=5; acc=10; vel_dif1=acc*dt; size1=10; 
    alpha2=1.0 ;  v_max2=5; acc=10; vel_dif2=acc*dt; size2=10; 
    robots = [ robots RobotClass([-50;0],[50;0],[0;0], 'red', alpha1, v_max,  vel_dif1, size1, 'circle')];
    robots = [ robots RobotClass([50;0],[-50;0],[-5;0], 'green', alpha2, v_max2, vel_dif1, size2, 'circle')];
    % robots = [ robots RobotClass([0;0],[0;0],[1;0], 'black', 0.5, 10, 20, 'circle')];

elseif EX==2
    % % 4*nmax ROBOTS IN A CIRCLE of Radius R
    nmax=4; R=60;  k_size=10; dt=0.1;
%     alpha1=0.5; alpha2=0.8; alpha3=0.2; alpha4=0.1;
    alpha1=0.5; alpha2=0.5; alpha3=0.5; alpha4=0.5;
    v_max=10;  acc=20; speed_dif=acc*dt; size=5; 
    for i=1:1:nmax
        x=R*cos(pi*i/(2*nmax)); y=R*sin(pi*i/(2*nmax));
        robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'red', alpha1, v_max, speed_dif, size, 'circle')];
        x=R*cos(pi*(i+nmax)/(2*nmax)); y=R*sin(pi*(i+nmax)/(2*nmax));
        robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'green', alpha2, v_max, speed_dif, size, 'circle')];
        x=R*cos(pi*(i+2*nmax)/(2*nmax)); y=R*sin(pi*(i+2*nmax)/(2*nmax));
        robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'yellow', alpha3, v_max, speed_dif, size, 'circle')];
        x=R*cos(pi*(i+3*nmax)/(2*nmax)); y=R*sin(pi*(i+3*nmax)/(2*nmax));
        robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'blue', alpha4, v_max, speed_dif, size, 'circle')];
    end

elseif EX==3
    % Robots placed in a square 
    dx=10; nx=3; ny=3; dy=10; size=2
    dt=0.1; v_max=5; k_size=10 ; %checks only neighbors k_times its size
    acc=10; speed_dif=acc*dt; % maximum allowable speed change in dt
    offset_X=100; offset_Y=50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y+20],[1 ; 0], 'red', 0.8, v_max, speed_dif, size, 'circle')];
        end
    end
    offset_X=-100; offset_Y=50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y+20],[1 ; 0], 'green', 0.5, v_max, speed_dif ,  size, 'circle')];
        end
    end
    offset_X=-100; offset_Y=-50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'blue', 0.5, v_max, speed_dif, size, 'circle')];
        end
    end
    offset_X=100; offset_Y=-50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'blue', 0.5, v_max,speed_dif, size, 'circle')];
        end
    end
    % obstacles
    robots = [ robots RobotClass([40;0],[40;0],[0 ; 0], 'black', -2, 0, 0, 12, 'circle')];
    robots = [ robots RobotClass([-40;0],[-40;0],[0 ; 0], 'black', -2, 0, 0, 12, 'circle')];
    robots = [ robots RobotClass([0;40],[0;40],[0 ; 0], 'black', -2, 0, 0, 12, 'circle')];
    robots = [ robots RobotClass([0;-40],[0;-40],[0 ; 0], 'black', -2, 0, 0, 12, 'circle')];
    robots = [ robots RobotClass([0;0],[0;0],[0 ; 0], 'black', -2, 0, 0, 12, 'circle')];

elseif EX==4
    % Robots placed in a line with a moving obstacle 
    dx=10; nx=2; ny=5; dy=15; size=2; alpha=0.5;
    dt=0.1; v_max=10; k_size=20 ; %checks only neighbors k_times its size
    acc=20; speed_dif=acc*dt; % maximum allowable speed change in dt
    offset_X=-80; offset_Y=50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=-j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;y],[5 ; 0], 'red',alpha,v_max,speed_dif, size, 'circle')];
        end
    end
    % the moving obstacle
        size=10; v_max= 1;
        robots = [ robots RobotClass([0;80],[0;-60],[-0 ; -11], 'blue', -2, v_max, speed_dif, size, 'circle')];
elseif EX==5
    dt=0.1; k_size=10;
    alpha=0.5 ;  v_max=20; acc=2000; vel_dif=acc*dt; size=5;
    
    robots = [ robots RobotClass([ 20;40],[-20;-60],[0 ; 0], 'green', alpha, v_max, vel_dif, size, 'circle')];
    robots = [ robots RobotClass([-20;40],[-40;-60],[0 ; 0], 'green', alpha, v_max, vel_dif, size, 'circle')];
    robots = [ robots RobotClass([00;10],[-00;-60],[0 ; 0], 'blue', alpha, v_max, vel_dif, size, 'circle')];
    robots = [ robots RobotClass([ 20;20],[20;-60],[0 ; 0], 'red', alpha,  v_max, vel_dif, size, 'circle')];
    robots = [ robots RobotClass([-20;20],[40;-60],[0 ; 0], 'red', alpha, v_max, vel_dif, size, 'circle')];
    
    robots = [ robots RobotClass([ 55;40],[55;40],[0 ; 0], 'black', -2, 0, 0, 20, 'circle')];
    robots = [ robots RobotClass([-55;40],[-55;40],[0 ; 0], 'black', -2, 0, 0, 20, 'circle')];
    robots = [ robots RobotClass([ 55;0],[55;0],[0 ; 0], 'black', -2, 0, 0, 20, 'circle')];
    robots = [ robots RobotClass([-55;00],[-55;00],[0 ; 0], 'black', -2, 0, 0, 20, 'circle')];
    robots = [ robots RobotClass([-20;-20],[-20;-20],[0 ; 0], 'black', -2, 0, 0, 20, 'circle')];
    robots = [ robots RobotClass([ 20;-20],[20;-20],[0 ; 0], 'black', -2, 0, 0, 20, 'circle')];
    
    for i=1:1:length(robots)
        robots(i).acc_onOff = 0 ;
    end
elseif EX==6
    % Robots placed in a square 
    dx=10; nx=3; ny=3; dy=10; size=2
    dt=0.1; v_max=10; k_size=15 ; %checks only neighbors k_times its size
    acc=20; speed_dif=acc*dt; % maximum allowable speed change in dt
    offset_X=100; offset_Y=50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y+20],[1 ; 0], 'red', 0.5, v_max, speed_dif, size, 'circle')];
        end
    end
    offset_X=-100; offset_Y=50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y+20],[1 ; 0], 'green', 0.5, v_max, speed_dif ,  size, 'circle')];
        end
    end
    offset_X=-100; offset_Y=-50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'blue', 0.5, v_max, speed_dif, size, 'circle')];
        end
    end
    offset_X=100; offset_Y=-50;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;-y],[1 ; 0], 'blue', 0.5, v_max,speed_dif, size, 'circle')];
        end
    end
    for i=1:1:length(robots)
        robots(i).acc_onOff = 0 ;
    end
    % obstacles
    square1 = Square([-30;25],[0;0],'yellow',25);
    square2 = Square([-30;-30],[0;0],'yellow',25);
    square3 = Square([+30;+25],[0;0],'yellow',25);
    square4 = Square([+30;-30],[0;0],'yellow',25);
    
elseif EX==7
        % Robots placed in a line with a moving obstacle 
    dx=10; nx=2; ny=5; dy=15; size=2; alpha=0.5;
    dt=0.1; v_max=5; k_size=20 ; %checks only neighbors k_times its size
    acc=20; speed_dif=acc*dt; % maximum allowable speed change in dt
    offset_X=-50; offset_Y=90;
    for i=1:1:nx
        for j=1:1:ny
            x=i*dx+offset_X;
            y=-j*dy+offset_Y;
            robots = [ robots RobotClass([x;y],[-x;y],[5 ; 0], 'red',alpha,v_max,speed_dif, size, 'circle')];
        end
    end
    % the moving obstacle
        square = Square([0;0],[0;6.5],'yellow',15);
end






%% plot my agents
N_robots=length(robots);
figure(1) 
title(strcat(num2str(N_robots),' Robot Agents Running on-',computer, ' (',num2str(rel_speed_aver),')'))
axis([-(Area_X+1) (Area_X+1) -(50+Area_Y+1) (50+Area_Y+1)])
text( -99, 90, [' Max Speed: ',num2str(v_max), ' m/s Accel: ',num2str(acc),' m/s^2 Sensor Range (k-size): ',num2str(k_size)],'FontSize',8);
hold on 
drawnow

flag_stop=1; %CC

% animation
N_steps=0;   %CC
N_colisions=0
%%%%%%%%%%%%%%%  BATCH MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%
if mod == 1
    N_robots=length(robots); flag_end=N_robots*(-2); %-2 is the priority when a robot reaches its target
    
    while flag_stop>flag_end
        
        tic %
         N_steps=N_steps+1;

         
        %collision detection
             
        % checks if there is any colission
        if collisionDetection(robots)==1
           N_colisions=N_colisions+1;
        end
        
        [robots,flag_stop]=execute_one_step(dt,robots,k_size);
if EX == 6
    for i=1:1:N_robots
        dist=norm(square1.position-robots(i).position)-square1.size;
        if dist < k_size*robots(i).Size
            [cones, flag_collision] = square1.get_cones_V3(robots(i).position,robots(i).Size,robots(i).velocity);
            robots(i).Cones = cones ;
            N_colisions=N_colisions+flag_collision;
        end
        dist=norm(square2.position-robots(i).position)-square2.size;
        if dist < k_size*robots(i).Size
            [cones, flag_collision] = square2.get_cones_V3(robots(i).position,robots(i).Size,robots(i).velocity);
            robots(i).Cones = [ robots(i).Cones cones] ;
            N_colisions=N_colisions+flag_collision;
        end
        dist=norm(square3.position-robots(i).position)-square3.size;
        if dist < k_size*robots(i).Size
            [cones, flag_collision] = square3.get_cones_V3(robots(i).position,robots(i).Size,robots(i).velocity);
            robots(i).Cones = [ robots(i).Cones cones] ;
            N_colisions=N_colisions+flag_collision;
        end
        dist=norm(square4.position-robots(i).position)-square4.size;
        if dist < k_size*robots(i).Size
            [cones, flag_collision] = square4.get_cones_V3(robots(i).position,robots(i).Size,robots(i).velocity);
            robots(i).Cones = [ robots(i).Cones cones] ;
            N_colisions=N_colisions+flag_collision;
        end
    end
end

if EX==7 
    for i=1:1:N_robots
        dist=norm(square.position-robots(i).position)-square.size;
        if dist < k_size*robots(i).Size
            [cones, flag_collision] = square.get_cones_V3(robots(i).position,robots(i).Size,robots(i).velocity);
            robots(i).Cones = cones ;
            N_colisions=N_colisions+flag_collision;
        end
    end
        square = square.update(dt);

end

step_time_plot(N_steps)=toc;    
fprintf('Number of steps %5d  #collisions %3d \n', N_steps, N_colisions); 
        drawnow
    %     pause(0.2)

    end
    exec_time = ceil(10*sum(step_time_plot))/10;
end

%%%%%%%%%%%%%%%  STEP MODE %%%%%%%%%%%%%%%%%%%%%%%%%%%
if mod == 2
   % THIS was FOR STEP BY STEP mode  
end

exec_time_rel=ceil(exec_time*rel_speed_aver*10)/10 ;
fprintf('simulation time %6.2f sec  N_steps %6.0f Step time %6.3f sec\n',N_steps*dt, N_steps, dt);
fprintf('Execution Time %6.2f , normalized to average %6.2f \n',exec_time, exec_time_rel);
% Copy the number of speed samples from Robot Class
N_samples=100; fntsz=8;

text(-99,-70, ['Iterrations     : ',num2str(N_steps)],'FontSize',fntsz);
text(-99,-80, ['Update time dt: ',num2str(dt), ' s'],'FontSize',fntsz);
text(-99,-90, ['Total time      : ',num2str(N_steps*dt), ' s'],'FontSize',fntsz);
text( 20,-70, ['# Speed Samples  :   ',num2str(N_samples)],'FontSize',fntsz);
text( 20,-80, ['Execution Time (norm) : ',num2str(exec_time),' s (',num2str(exec_time_rel),')'],'FontSize',fntsz);
text( 20,-90, ['ExecTime/Robot (norm): ',num2str(ceil(10*(exec_time/N_robots))/10),' s (',num2str(ceil(100*exec_time_rel/N_robots)/100),')'],'FontSize',fntsz);

% figure(2) 
% step_time_plot(N_steps+1)=0;
% plot(step_time_plot);
% title('Execution Time per simulation step (secs)');
% text( 10, 0.95*max(step_time_plot), [num2str(N_robots),' Robots, Max Speed: ',num2str(v_max), ' m/s Accel: ',num2str(acc),' m/s^2 Sensor Range (k-size): ',num2str(k_size)],'FontSize',fntsz);
