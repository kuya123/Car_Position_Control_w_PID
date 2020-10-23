clear all, close all, clc

m = 1;
M = 1;
L = 2;
g = -10;
d = 1;

###############################################################
##### section 1 analysis stability #########
###############################################################
display("\n");
display("********* analyze system stability @ pointing UP ******");
[A B C D]= proj_SS_inverted_pendulum_car_SS_model(m,M,L,g,d,0);
display (A);
display ("eigen value ::\n");
display(eig(A));

display("\nWhether this Xdot = Ax + Bu is controllable ? \n")
det(ctrb(A,B))

display("\nResult : eigen value value of A at UP position is not stable. The system is controllable I need to construct a feedback -K to make it stable");


###############################################################
###### setcion 2  : try POLE placement method #################
###############################################################
pole=[ -2 -3 -2.5 -4]';
display("\nFeedback coefficeient :: \n");
K=place(A,B,pole)

#----- simulate this new feedback system with ODE -----
tspan=0.1:0.1:20;

y0=[-2;0;0;0];
y_target=[2;0;0;0];
[t1,y_new1] = ode45(@(t,y)proj_SS_inverted_pendulum_car_ODE_model(y,m,M,L,g,d,-K*(y_target-y)),tspan,y0);


y0=y_new1(numel(t1),:)';
y_target=[-3;0;0;0];
[t2,y_new2] = ode45(@(t,y)proj_SS_inverted_pendulum_car_ODE_model(y,m,M,L,g,d,-K*(y_target-y)),tspan,y0);

y_overall=[y_new1;y_new2];
t=[t1;t2+t1(numel(t1))];

figure();
plot(t,y_overall);
title ("Movement with Pole Placement K");
legend("x","v","the","the-a");



####################################################################
###### setcion 3  : try LQR  method ################################
####################################################################
## linear quadratic regulator
Q=[50 0 0 0;0 1 0 0; 0 0 2 0; 0 0 0 1];
R=0.01;
K=lqr(A,B,Q,R);


tspan=0.1:0.1:20;

y0=[-2;0;0;0];
y_target=[2;0;0;0];
[t1,y_new1] = ode45(@(t,y)proj_SS_inverted_pendulum_car_ODE_model(y,m,M,L,g,d,-K*(y_target-y)),tspan,y0);


y0=y_new1(numel(t1),:)';
y_target=[-3;0;0;0];
[t2,y_new2] = ode45(@(t,y)proj_SS_inverted_pendulum_car_ODE_model(y,m,M,L,g,d,-K*(y_target-y)),tspan,y0);

y_overall=[y_new1;y_new2];
t=[t1;t2+t1(numel(t1))];

figure();
plot(t,y_overall);
title ("Movement with LQR K");
legend("x","v","the","the-a");


###### simulate the flow by catoon ####

% comment out below annimation due to time saving.
%{
figure();
for k=1:1:numel(y_overall)
    proj_SS_PID_draw_inverted_pendulum(y_overall(k,:),m,M,L)
endfor
%}


#{

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


#}
