

function [A B C D]=proj_SS_inverted_pendulum_car_SS_model(m,M,L,g,d,positionangle)
% proj_SS_inverted_pendulum_car_SS_model: 
% positionangle = 0 pendulum points up at 0 degree
% positionangle = pi pendulum points down at pi degree

% but i need to be careful that, this positionangle flag is only used to determine the fix point linear system A B C D .  for simulation, we need include the delta. 

if (positionangle==0)
    x=-1;
elseif (positionangle==pi)
    x=1;
endif

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 x*d/(M*L) x*(m+M)*g/(M*L) 0];
    
B = [0; 1/M; 0; -1*x/(M*L)];
C = eye(4);
D = 0*B;
