function [x, P] = tu_q(x, P, T, Rw)
% Task 4: Time update function of the EKF
% x         Last time step mean
% P         Last time step covariance matrix
% omega     Measured angular rate
% T         Time since last measurement
% Rw        Process noise covariance matrix


S_q = Sq(x);

F = eye(4) ;
G = (T/2)*(S_q);

x  = F*x;

P = F*P*F.' + G*Rw*G.';

end