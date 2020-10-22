clear all, close all, clc

m = 1;
M = 1;
L = 2;
g = -10;
d = 1;


##### section 1 analysis stability #########
display("\n\n\n");
display("********* analyze system stability @ pointing UP ******");
[A B C D]= proj_SS_inverted_pendulum_car_SS_model(m,M,L,g,d,0);
display (A);
display ("eigen value");
display(eig(A));

display("\n\n\n");
display("********* analyze system stability @ pointing DOWN ******");
[A B C D]= proj_SS_inverted_pendulum_car_SS_model(m,M,L,g,d,pi);
display (A);
display ("eigen value");
display(eig(A));


# base on above eig value, we can see pi position is stable position, thus we can use SS model around this case for time transient simulation







###### section 2 simulate SS model around equilirum point pi
ss_sys=ss(A,B,C,D);

display("\n\n");
display("state space system model for PI angle :\n");
display(ss_sys);

tspan=.1:.1:3;
X0=[0,0,0.8,0];   

## the model has decide dimension of B is 4x1, thus our U only can be 1 row vector, thus the u must be one colume.

u=zeros(numel(tspan),1);  

[y t x]=lsim(ss_sys, u, tspan, X0);


% above simulation is around equilibrium point, thus I need add deta angle of pi before drawing.
ynl=y+[zeros(numel(t),2) ones(numel(t),1)*pi zeros(numel(t),1)];

% comment out below annimation due to time saving. 
%{
for k=1:1:numel(t)
    proj_SS_PID_draw_inverted_pendulum(ynl(k,:),m,M,L)
endfor
%}





###### section 3 simulate SS model around equilirum point 0 ( upwards )
###### check ss_sys controllability and observbiltiy #####

[A B C D]= proj_SS_inverted_pendulum_car_SS_model(m,M,L,g,d,0);
ss_sys=ss(A,B,C,D);

display("\n\n");
display("state space system model for ZERO angle :\n");
display(ss_sys);


display("\n\n");
display("controllability and observability check\n");
display(["controllability matrix -->  det " num2str(det(ctrb(A,B))) " rank " num2str(rank(ctrb(A,B)))]);


C=[1 0 0 0]; % check whether measure distance along can reconstruct the system.
display(["observability matrix -->  det " num2str(det(obsv(A,C))) " rank " num2str(rank(obsv(A,C)))])



