# G is the transfer function of the system.
# K_friction is the friction coefficient
function G = proj_PID_car_TF_model(m,Kcoeff,K_friction)
syms s;
G=Kcoeff*tf(1, m*[1 K_friction 0]);
#Gd=sym2poly(m*(s*s+s*K_friction));
#G=tf(1,Gd );
