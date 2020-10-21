m=0.05;
force=[0 0.8 0 0 0];
k=0.5;

warning('off', 'all');

#################  ODE linear simulation ##############

tspan = 0:.01:10;

f=force(1);
y0=[0,0];
[t,yNL1] = ode45(@(t,y)proj_PID_car_ODE_model(y,m,f,k),tspan,y0);


y1=yNL1(1001,:);
f=force(2);

[t,yNL2] = ode45(@(t,y)proj_PID_car_ODE_model(y,m,f,k),tspan,y1);


y2=yNL2(1001,:);
f=force(3);
[t,yNL3] = ode45(@(t,y)proj_PID_car_ODE_model(y,m,f,k),tspan,y2);

y3=yNL3(1001,:);
f=force(4);
[t,yNL4] = ode45(@(t,y)proj_PID_car_ODE_model(y,m,f,k),tspan,y3);

y4=yNL4(1001,:);
f=force(5);
[t,yNL5] = ode45(@(t,y)proj_PID_car_ODE_model(y,m,f,k),tspan,y4);


tnew= [tspan tspan+10.01 tspan+20.02 tspan+30.03 tspan+40.04];

yout=[yNL1' yNL2' yNL3' yNL4' yNL5']';




figure;

plot(tnew, yout(:,1));
xlim([0 50]);
#figure
#plot(tnew, yout(:,2));


################### Transfer function  memthod #############


G=proj_PID_car_TF_model(m,k);

t=0:0.01:50.04;
f1 = ones(1001,1)*force(1);
f2 = ones(1001,1)*force(2);
f3 = ones(1001,1)*force(3);
f4 = ones(1001,1)*force(4);
f5 = ones(1001,1)*force(5);

fnew = [f1' f2' f3' f4' f5']';

lsim(G, fnew, t)
