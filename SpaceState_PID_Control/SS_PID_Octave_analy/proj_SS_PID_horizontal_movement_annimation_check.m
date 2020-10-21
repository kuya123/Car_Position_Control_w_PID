
%% this code is to simulate a horizontal movement of the car.  the purpose is to check the function of drawing block

m=1;
M=1.5;
L=1;

for k=-0.5:0.1:0.5
 proj_SS_PID_draw_inverted_pendulum([k 0 0.5 0],m,M,L)
endfor
