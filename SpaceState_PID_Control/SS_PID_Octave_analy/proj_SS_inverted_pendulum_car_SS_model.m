function SS_sys=proj_SS_inverted_pendulum_car_SS_model(m,M,L,g,d)
A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 d/(M*L) (m+M)*g/(M*L) 0];
B = [0; 1/M; 0; -1/(M*L)];
C = eye(4);
SS_sys = ss(A,B,C,0*B);