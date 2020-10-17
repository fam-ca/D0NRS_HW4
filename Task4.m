clear all
close all
p0 = [1 0 1];
pf = [sqrt(2)/2 sqrt(2)/2 1.2];

v_max = 1; %maximum linear velocity, m/s
a_max = 10;%maximum linear acceleration, m/s^2
freq = 100;
dt = 1/freq;
q0 = IK(p0(1),p0(2),p0(3));
qf = IK(pf(1),pf(2),pf(3));
ts = v_max/a_max;
tf = (qf-q0)/v_max+ts;
tspan = 0:dt:max(tf);
tf = max(tf);
% positions
x = ((pf(1)-p0(1))/tf).*tspan+p0(1);
y = ((pf(2)-p0(2))/tf).*tspan+p0(2);
z = ((pf(3)-p0(3))/tf).*tspan+p0(3);

% velocities
vx = ((pf(1)-p0(1))/tf);
vy = ((pf(2)-p0(2))/tf);
vz = ((pf(3)-p0(3))/tf);
vel = [vx;vy;vz];

N = length(x);
%for each waypoint we compute the joint configurations using IK
%then we compute the joint velocities using Jacobian
for i=1:N
    jointConfig(i,:) = IK(x(i), y(i), z(i));
    if i == 1 || i==N
        jointVel(i,:) = [0 0 0];
    else
        J = Task1_Jacobian(jointConfig(i,1),jointConfig(i,2),jointConfig(i,3));
        jointVel(i,:) =  (J(1:3,:)\vel)';
    end
end

% joint trajectory for each joint
n = 20;
Q = [];
Qd = [];
for i=1:N-1
    t1 = tspan(i); t2 = tspan(i+1);
    A = [1 t1 t1^2 t1^3
        0 1 2*t1 3*t1^2
        1 t2 t2^2 t2^3
        0 1 2*t2 3*t2^2];
    for j = 1:3
         q1 = jointConfig(i,j); q2 = jointConfig(i+1,j);
         v1 = jointVel(i,j); v2 = jointVel(i+1,j);
         c = [q1;v1;q2;v2];
         b = A\c;
         t = linspace(t1,t2,n);
         q_n(j,:) = b(1)+b(2).*t+b(3).*t.^2+b(4).*t.^3;
         qd_n(j,:) = b(2)+2*b(3).*t+3*b(4).*t.^2;
    end   
    Q = [Q q_n];
    Qd = [Qd qd_n];
end
Traj = FK(Q);

%plot the graphs
ns = n*N-n;
T = linspace(0,tf,ns);
figure;
plot(tspan, x, T, Traj(1,:),'k--','linewidth',1);
grid on
legend('Theoretical x','Actual x')
figure
plot(tspan, y, T, Traj(2,:),'k--','linewidth',1);
grid on
legend('Theoretical y','Actual y','location','northwest')
figure
plot(tspan, z, T, Traj(3,:),'k--','linewidth',1);
grid on
legend('Theoretical z','Actual z','location','northwest')

figure
subplot(311)
plot(T, Q(1,:))
ylabel('Joint 1')
grid on
subplot(312)
plot(T, Q(2,:))
ylabel('Joint 2')
grid on
subplot(313)
plot(T, Q(3,:))
ylabel('Joint 3')
grid on

% visualization
L1 = 1;
L2 = 1;
L3 = 1;
q1 = jointConfig(1,1);
q2 = jointConfig(1,2);
q3 = jointConfig(1,3);

figure;
plot3(x,y,z,'r','linewidth',1)
hold on
plot3(Traj(1,:),Traj(2,:),Traj(3,:),'k--','linewidth',1)
line([0 0],[0 0],[0 L1],'linewidth',2,'Color','black');
line([0 -L2*cos(q2)*sin(q1)], [0 L2*cos(q1)*cos(q2)], [L1 L1 + L2*sin(q2)],'linewidth',2,'Color','black');
line([-L2*cos(q2)*sin(q1) -sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2))],...
    [L2*cos(q1)*cos(q2) cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2))],...
    [L1 + L2*sin(q2) L1 + L3*sin(q2 + q3) + L2*sin(q2)],'linewidth',2,'Color','black');
plot3(0,0,0,'r*','linewidth',4,'MarkerSize',10)
plot3(0,0,L1,'r*','linewidth',4,'MarkerSize',10)
plot3( -L2*cos(q2)*sin(q1), L2*cos(q1)*cos(q2),L1 + L2*sin(q2),'r*','linewidth',4,'MarkerSize',10)
plot3(-sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2)), cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2)),L1 + L3*sin(q2 + q3) + L2*sin(q2),'r*','linewidth',4,'MarkerSize',10 )
q1 = jointConfig(end,1);
q2 = jointConfig(end,2);
q3 = jointConfig(end,3);

line([0 0],[0 0],[0 L1],'linewidth',2,'Color','green');
line([0 -L2*cos(q2)*sin(q1)], [0 L2*cos(q1)*cos(q2)], [L1 L1 + L2*sin(q2)],'linewidth',2,'Color','green');
line([-L2*cos(q2)*sin(q1) -sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2))],...
    [L2*cos(q1)*cos(q2) cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2))],...
    [L1 + L2*sin(q2) L1 + L3*sin(q2 + q3) + L2*sin(q2)],'linewidth',2,'Color','green');
plot3( -L2*cos(q2)*sin(q1), L2*cos(q1)*cos(q2),L1 + L2*sin(q2),'r*','linewidth',4,'MarkerSize',10)
plot3(-sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2)), cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2)),L1 + L3*sin(q2 + q3) + L2*sin(q2),'r*','linewidth',4,'MarkerSize',10 )
title('Visualization');
xlabel('x')
ylabel('y')
zlabel('z')