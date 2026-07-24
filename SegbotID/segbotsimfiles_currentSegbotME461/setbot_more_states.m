g = 9.81;
m = 0.0615;
R = 0.067;
J_w = (m*R^2)/2;
M = 0.5764;
L = 0.049;
J_psi = (M*L^2)/3;
J_m = 2.285e-4;
R_m = 1.875;
K_b = 0.3907;
K_t = 0.3907;
n = 1;
f_m = 0.0022;
alpha = (n*K_t)/R_m;
beta = ((n*K_t*K_b)/R_m) + f_m;
a = 2*pi*12.5;

D = [(M*L^2 + J_psi + 2*n^2*J_m) (M*L*R - 2*n^2*J_m);
    (M*L*R - 2*n^2*J_m) ((2*m+M)*R^2 + 2*J_w + 2*n^2*J_m)];
b = [M*g*L -2*beta 0 2*beta;
    0 2*beta 0 -2*beta];
h = [-2*alpha;
    2*alpha];
c = [1 1/a 0 0 0; 0 1 1/a 0 0;0 0 0 1 0; 0 0 0 0 1]; % matrix for psi in F_psi and F_theta
d = [0 0 1 0 0; 0 0 0 0 0];


Dib = D\b;
Dih = D\h;
Dibc = Dib*c;
D_new = a*(Dibc - d);

A5 = [0 1 0 0 0; 
    0 0 1 0 0; 
    D_new(1,:);
    0 0 0 0 1;
    D_new(2,:);];
B5 = -[0;0;a*Dih(1);0;a*Dih(2);];

A4 = [0 1    0 0;
    Dib(1,:);
    0 0 0 1;
    Dib(2,:)];
B4 = [0;
    Dih(1);
    0;
    Dih(2)];
A = [A5(1,1) A5(1,2) A5(1,3) A5(1,5);
    A5(2,1) A5(2,2) A5(2,3) A5(2,5);
    A5(3,1) A5(3,2) A5(3,3) A5(3,5);
    A5(5,1) A5(5,2) A5(5,3) A5(5,5)];
B = [B5(1);
    B5(2);
    B5(3);
    B5(5)];
% K = [-30,-2.8,-1]*6/10  % 6/10 because Simulation max is 6V and real system max is 10 for pwm output
% K = [-60,-4.5,-1];
K = place(A,B,[-800,-9,-10,-11])