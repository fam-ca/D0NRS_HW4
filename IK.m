function Q  = IK(x,y,z)

L1 = 1;
L2 = 1;
L3 = 1;
% x = 0;
% y = 2;
% z = 1;
d = sqrt(x^2+y^2+(z-L1)^2);
q1 = atan2(-x,y);
q3 = acos((d^2-L2^2-L3^2)/(2*L2*L3));
alpha = -atan2(2*L2*L3*sin(q3),L2^2-L3^2+d^2);
beta = atan2(z-L1,sqrt(x^2+y^2));
% if q3>0
%     m = -1;
% else
%     m = 1;
% end
m = 1;
q2 = m*alpha+beta;

Q = [q1,q2,q3];


end