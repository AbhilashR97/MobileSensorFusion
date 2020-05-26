function [x, P] = mu_m(x, P, mag, m0, Rm) 
% EKF update using the magnetometer measurements

% mag       Measured magnetic field vector
% Rm        Measurement noise covariannce matrix
% m0        nominal magnetic field

Q = Qq(x);
[dQ0, dQ1, dQ2, dQ3] = dQqdq(x);

hx = Q'*m0;
Hx = [dQ0'*m0, dQ1'*m0, dQ2'*m0, dQ3'*m0];

% P_temp = kron(eye(3), P);

S = Hx*P*Hx.' + Rm;
S = (S + S')/2;

K = P*Hx.'*inv(S);

x = x + K*(mag - hx);

P = P - K*S*K.';


end