BalRob_Params;
% State Matrices  
M_lin = [M11, M12_lin;
         M12_lin, M22];
G = [0, 0;
    0, g22];

A = [zeros(2), eye(2);
    -inv(M_lin)*G -inv(M_lin)*Fv_pr];

B = (2*gbox.N*mot.Kt/mot.R)*[zeros(2); inv(M_lin)]*[1; -1];

C = [1 0 0 0];

% continous-time system
csys = ss(A, B, C, 0);

% discrete-time system
dsys = c2d(csys,1e-2);

Phi = get(dsys, 'A');
Gamma = get(dsys, 'B');
H = C;

%Nominal Tracking Matrices
N_mat = [Phi - eye(4), Gamma;
         H, 0];

N_res = linsolve(N_mat, [0, 0, 0, 0, 1].');

Nx = N_res(1:4);
Nu = N_res(5);

% LQR costs

Q = diag([(18/pi)^2; (360/pi)^2; 0; 0]);
r = 1;
ro = 500; 

% K LQR
K_Nom_LQR_Br = dlqr(Phi, Gamma, Q, ro*r); 


