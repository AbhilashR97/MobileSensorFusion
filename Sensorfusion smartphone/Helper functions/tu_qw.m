function [x, P] = tu_qw(x, P, omega, T, Rw)
% Task 4: Time update function of the EKF
% x         Last time step mean
% P         Last time step covariance matrix
% omega     Measured angular rate
% T         Time since last measurement
% Rw        Process noise covariance matrix


S_w = Somega(omega);
S_q = Sq(x);

F = (eye(4) + (T/2)*(S_w));
G = (T/2)*(S_q);

x  = F*x;

P = F*P*F.' + G*Rw*G.';

end