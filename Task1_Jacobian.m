function J = Task1_Jacobian(q1,q2,q3)
% syms q1 q2 q3
L1 = 1;
L2 = 1;
L3 = 1;
% syms L1 L2 L3
H = Rz(q1)*Tz(L1)*Rx(q2)*Ty(L2)*Rx(q3)*Ty(L3);
% rotation matrix
R = H(1:3,1:3);
invR = [R^(-1) zeros(3,1); 0 0 0 1];
% diff H by q1
Hd = Rzd(q1)*Tz(L1)*Rx(q2)*Ty(L2)*Rx(q3)*Ty(L3)*invR;
% extract elements from Hd and get the first column of Jacobian matrix
J1 = [Hd(1:3,4); Hd(3,2); Hd(1,3); Hd(2,1)];
% diff H by q2
Hd = Rz(q1)*Tz(L1)*Rxd(q2)*Ty(L2)*Rx(q3)*Ty(L3)*invR;
% extract elements from Hd and get the second column of Jacobian matrix
J2 = [Hd(1:3,4); Hd(3,2); Hd(1,3); Hd(2,1)];
% diff H by q3
Hd = Rz(q1)*Tz(L1)*Rx(q2)*Ty(L2)*Rxd(q3)*Ty(L3)*invR;
% extract elements from Hd and get the third column of Jacobian matrix
J3 = [Hd(1:3,4); Hd(3,2); Hd(1,3); Hd(2,1)];
% J = [simplify(J1) simplify(J2) simplify(J3)]
J = [J1 J2 J3];
end