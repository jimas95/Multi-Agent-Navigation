clear all;
size = 5;
Area_X = 15;
Samples = 350;
figure(1)


subplot(1,2,2)
[circlex circley] = rand_circ(Samples,0,0,size);
plot(circlex,circley,'o');
title('circle distributed sampling')
axis([-(Area_X+1) (Area_X+1) -(Area_X+1) (Area_X+1)])

subplot(1,2,1)
theta = rand(1,Samples)*2*pi;
r = size*rand(1,Samples);
x = (r).*cos(theta);
y = (r).*sin(theta);
plot(x,y,'o');
title('circle sampling')
axis([-(Area_X+1) (Area_X+1) -(Area_X+1) (Area_X+1)])