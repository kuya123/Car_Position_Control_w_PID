
# build the system
m=0.5;
M=1.5;
L=1;
g=9.8;
d=0.5;

y0=[0 0 1.4 0];
tspan=0.1:0.1:3;
force=[0.8 -0.8 0.8 -0.8];
Y=[];


for p=1:1:4
    u=force(p);
    [t,y_new] = ode45(@(t,y)proj_SS_inverted_pendulum_car_ODE_model(y,m,M,L,g,d,u),tspan,y0);
    
    y0=y_new(numel(t),:);
    Y=[Y' y_new']';
endfor


T=0.1:0.1:12;
figure;
hold on;
plot(T,Y(:,1));
plot(T,Y(:,2));
plot(T,Y(:,3));
legend ("location", "speed","angle");


for T=1:1:120
 proj_SS_PID_draw_inverted_pendulum(Y(T,:),m,M,L)
endfor
