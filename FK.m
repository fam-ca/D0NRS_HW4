function Pos = FK(Q)
x = []; y = []; z = [];
q1 = Q(1,:);
q2 = Q(2,:);
q3 = Q(3,:);
L1 = 1;
L2 = 1;
L3 = 1;
for i =1:length(Q)
    H = Rz(q1(i))*Tz(L1)*Rx(q2(i))*Ty(L2)*Rx(q3(i))*Ty(L3);
    x(i) = H(1,4);
    y(i) = H(2,4);
    z(i) = H(3,4);
end
Pos = [x; y; z];

% syms L1 L2 L3 q1 q2 q3
% H = simplify(Rz(q1)*Tz(L1)*Rx(q2)*Ty(L2)*Rx(q3)*Ty(L3))
