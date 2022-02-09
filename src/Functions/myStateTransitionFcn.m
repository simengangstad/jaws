function x = myStateTransitionFcn(x,u)

% Low frequency control plant
M=[7.0101e6 0 0;
   0 7.8784e6 -2.5955e6;
   0 -2.5955e6 3.57e9];
D=[2.6486e5 0 0;
   0 8.8164e5 0;
   0 0 3.3774e8];

% Tuning Parameters for control plant
Omega=diag([2*pi/9,2*pi/9,2*pi/9]);
Lambda=diag([0.07,0.07,0.07]);
K_omega=diag([0.5,0.5,0.01]);

% Bias model
T_b=diag([1000,1000,1000]);

% State space form matrices of WF motions
A_omega=[zeros(3,3) eye(3,3);
         -Omega^2 -2*Lambda*K_omega];
B=[zeros(12,3);
    inv(M)];

% Sampling time
T_s=0.1;

% Rotation Matrix
psi=x(9);
R=[cos(psi) -sin(psi) 0;
   sin(psi) cos(psi)  0;
   0        0         1];

% Discretization (Forward Euler)
x=x+T_s*([A_omega*x(1:6);
           R*x(13:15);
           -inv(T_b)*x(10:12);
           -inv(M)*D*x(13:15)+inv(M)*R'*x(10:12)]+B*u);

end
