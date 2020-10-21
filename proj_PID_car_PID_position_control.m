clc ; clear; 

m=0.55;
K_friction=0.5;  # friction coefficient
Kcoeff=0.01; # -- the coefficient that translate input error to force

Kp=30;
Ki=0;
Kd=20;



############# System Building ########################

# feedback system
#{

 r(ref) --- sum {E(error)}---- Kp --- {u (force)}-- PLANT ------ X
             |---------------------------------------------|

#}
display("\n\n");
display("****************************\n");
display("****** System PID control analysis *****\n");
display("****************************\n");

display("\n\n");
display("******* PID parameters **********\n");
Kp
Ki
Kd



# system PLANT
syms s;
G=proj_PID_car_TF_model(m,Kcoeff,K_friction);
display("\n\n***** PLANT Original TF:");
G

G_control=proj_PID_car_PID_control_blk_TF_model(Kp,Ki,Kd);
#G_control


G_forward=G_control*G;
display("\n\n***** Controlled System TF:");
G_forward

G_feekback = feedback(G_forward, 1);
#G_feekback

############### Simulation ###########################


display("\n\n***** system gain margin and phase margin")
[GM PM FGM FPM]= margin(G_feekback)

display("\n\n***** system sensitivity margin :Ms")
[Ms Frequency]=sensitivity(G_forward)

display("\n\n***** poles and zeros")
pole_location=pole(G_feekback)
zero_location=zero(G_feekback)

# Display system stability 
figure;
bode(G_feekback);

figure;
pzmap(G_feekback);




r = [ zeros(5,1)' ones(195,1)']*5;
t=0:0.25:49.9;
[y t]=lsim(G_feekback, r, t);
figure;
h=plot (t,y);

display("\n\n***** system final value")
display(["final value :" num2str(y(numel(t)))]);


%{
for k=1:1:numel(t)
  proj_PID_draw_simple_car(y(k));
endfor
%}