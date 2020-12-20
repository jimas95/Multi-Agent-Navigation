close all
clear all
cones = Cone([10;8],[5;5],[5;-5],[0;0],[5;0],3,5);

line = animatedline;

figure(1) 
Area = 10 ;
title("cone test")
axis([-(Area+1) (Area+1) -(Area+1) (Area+1)])
hold on 

addpoints(line,cones.Q1(1),cones.Q1(2));
addpoints(line,cones.Q2(1),cones.Q2(2));
addpoints(line,cones.Q3(1),cones.Q3(2));
addpoints(line,cones.Q1(1),cones.Q1(2));
drawnow;

% N_points=8000;
% x=rand(1,N_points)*20-10;
% y=rand(1,N_points)*20-10;
[x,y] = rand_circ(1000,4,5,7);

point = [0;0];
plot(point(1),point(2),'o')
for i=1:1:length(x)
        if ~cones.isIn([point(1)+x(i);point(2)+y(i)])
            plot(point(1)+x(i),point(2)+y(i),'o')
        end
end
    

function [X,Y] = rand_circ(N,x,y,r)
% Generates N random points in a circle.
% RAND_CIRC(N) generates N random points in the unit circle at (0,0).
% RAND_CIRC(N,x,y,r) generates N random points in a circle with radius r 
% and center at (x,y).
if nargin<2
   x = 0;
   y = 0;
   r = 1;
end
Ns = round(1.28*N + 2.5*sqrt(N) + 100); % 4/pi = 1.2732
X = rand(Ns,1)*(2*r) - r;
Y = rand(Ns,1)*(2*r) - r;
I = find(sqrt(X.^2 + Y.^2)<=r);
X = X(I(1:N)) + x;
Y = Y(I(1:N)) + y;
end
            