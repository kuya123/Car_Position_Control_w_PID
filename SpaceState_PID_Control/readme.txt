State space design thinking flow

1. Define system parameters

# ODE check
2. Define system model by ODE basing on physical equation. ( ODE can discribe non-stable non linear system behavior): y vector [x dot-x theta  dot-theta]
3. Do a simple time transient simulation basing on ODE.  

# SS check
4. Define linearized state space system model : A B C D  around theta = pi  and 0 (downwards direction)
5. understand its stability by eigen value
6. simulate its time transient 

# control check
7. check controllability and observability
8. in our case we can measure distance and angle, thus we can temporarily bypass observability
9. select target eigen value and calculate target Kp with LQR filter.
10. evaluate new feedback system stability and time transient 
11. study system type and decide whether we need a Ki
12. improve system by introducing Kd and simulate again. 

I hope I can create out a good demo car by following above procedure.
