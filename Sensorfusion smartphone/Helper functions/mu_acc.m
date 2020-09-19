function [x, P] = mu_acc(x, P, yacc, Ra, g0)
% EKF update using the accelerometer measurements

% yacc      measured acceleration vector
% Ra        Measurement noise covariannce matrix
% g0        nominal gravity vector

Q = Qq(x);
[dQ0, dQ1, dQ2, dQ3] = dQqdq(x);

hx = Q'*g0;
Hx = [dQ0'*g0, dQ1'*g0, dQ2'*g0, dQ3'*g0];

% P_temp = kron(eye(3), P);

S = Hx*P*Hx.' + Ra;
S = (S + S')/2;

K = P*Hx.'*inv(S);

x = x + K*(yacc - hx);

P = P - K*S*K.';

end