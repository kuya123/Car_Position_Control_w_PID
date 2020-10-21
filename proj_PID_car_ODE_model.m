# y vector : [x, x-]   x- is speed, x is location
# m is mass
# f is force
# K_friction is the friction coefficient
function dy = proj_PID_car_ODE_model(y,m,f,K_friction)

dy(1,1)=y(2);
dy(2,1)=f/m-K_friction*y(2);   # first , we dont consider fricttion, since it will result into some kind step responce. 

