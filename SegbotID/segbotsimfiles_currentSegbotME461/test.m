% g = 9.81;
% m = 0.0615;
% R = 0.0575;
% J_w = (m*R^2)/2;
% M = 0.5764;
% L = 0.049;
% J_psi = (M*L^2)/3;
% J_m = 2.285e-4;
% R_m = 12/1.5;
% K_b = 0.3907;
% K_t = 0.3907;
% n = 1;
% f_m = 0.0022;
% alpha = (n*K_t)/R_m;
% beta = ((n*K_t*K_b)/R_m) + f_m;

g = 9.81;
m = 0.05; %34.4+13.9+gears(estimate)
R = 0.05966;%119.32
J_w = (m*R^2)/2;
M = 0.845;%945-50*2
L = 0.07475;
J_psi = (M*L^2)/3;
J_m = 2.285e-4;
R_m = 12/1.5;
K_b = 0.3907;
K_t = 0.3907;
n = 1;
f_m = 0.0022;
a = 2*pi*12.5*3;
alpha = (n*K_t)/R_m;
beta = ((n*K_t*K_b)/R_m) + f_m;


h = [-2*alpha*a;
    2*alpha*a];


D = [(M*L^2 + J_psi + 2*n^2*J_m) a*(M*L*R - 2*n^2*J_m);
    (M*L*R - 2*n^2*J_m) a*((2*m+M)*R^2 + 2*J_w + 2*n^2*J_m)];

c2 = m*g*L-2*beta*a;
c3 =-a*(M*L^2+J_psi+2*n^2*J_m)-2*beta;
M1 = [a*m*g*L c2 c3 0 0];
c1 =  -a*(M*R*L-2*n^2*J_m)+2*beta;
M2 = [0 2*a*beta c1 0 -2*a*beta];


b = [M1;M2];
D_new = D\b;
A5 = [0 1 0 0 0; 
    0 0 1 0 0; 
    D_new(1,:);
    0 0 0 0 1;
    D_new(2,:);];
Dih = D\h;
B5 = [0;0;Dih(1);0;Dih(2);];
A = [A5(1,1) A5(1,2) A5(1,3) A5(1,5);
    A5(2,1) A5(2,2) A5(2,3) A5(2,5);
    A5(3,1) A5(3,2) A5(3,3) A5(3,5);
    A5(5,1) A5(5,2) A5(5,3) A5(5,5)];
B = [B5(1);
    B5(2);
    B5(3);
    B5(5)];
% % K = [-30,-2.8,-1]*6/10  % 6/10 because Simulation max is 6V and real system max is 10 for pwm output
    K = [-60,-4.8,-0.25,-1];
K1 = [-60,-4.8,0,-1];

k1 = eig(A-B*K);
% k2 = eig(A-B*K1)
% K = place(A,B,[-2800 -18 -15 -2])
Q = eye(4);
Q(1,1) = 100000;
% Q(3,3) = 0.00001;
R = 0.05;
K = lqrd(A,B,Q,R,0.004)
