Nominal_Tracking;

%Robust Tracking Matrices
Phi_e = [1, H;
         zeros(4,1), Phi];
Gamma_e = [0; Gamma];

Q_rob = diag([0.1; (18/pi)^2; (360/pi)^2; 0; 0]);


K_Rob = dlqr(Phi_e, Gamma_e, Q_rob, ro*r); 

Ki = K_Rob(1);
% Ki = -1.6;
K_Rob_LQR = K_Rob(2:5);