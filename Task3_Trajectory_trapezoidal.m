clear all;
close all;

q0 = [0 0 0];
qf = [2 3 4];
v0 = 0;
v_max = 1; %rad/s
a_max = 10; %rad/s^2
freq = 100; %Hz
ts = v_max/a_max;

dt = 1/freq;
n = 0;
while(floor(dt*10^n)~=dt*10^n)
    n = n+1;
end
E = 10^-n;
if rem(ts,dt)~=0
    ts_new = round(ts,n)+E;
else
    ts_new = round(ts,n);
end
tf = (qf-q0)/v_max+ts_new;
if rem(tf(1),dt)~=0
    tf_new = round(tf,n)+E;
else
    tf_new = round(tf,n);
end

ts_new = max(ts_new);
tf_new = max(tf_new);
v_new = (qf-q0)/(tf_new-ts_new);
a_new = v_new/ts_new;

%from 0 to ts
a10 = q0;
a11 = [v0 v0 v0];
a12 = [0.5*a_new 0.5*a_new 0.5*a_new];
%from ts to tf-ts
a20 = q0+0.5*a_new.*ts_new.^2+(v0-v_new).*ts_new;
a21 = [v_new v_new v_new];
%from tf-ts to tf
a30 = qf-0.5*a_new.*tf_new.^2;
a31 = a_new.*tf_new;
a32 = [-0.5*a_new -0.5*a_new -0.5*a_new];

t_new = 0:dt:tf_new;
q1_new = (a10(1)+a11(1).*t_new+a12(1).*t_new.^2).*(t_new<=ts_new)+...
    (a20(1)+a21(1).*t_new).*(t_new>ts_new).*(t_new<tf_new-ts_new)+...
    (a30(1)+a31(1).*t_new+a32(1).*t_new.^2).*(t_new>=tf_new-ts_new).*(t_new<=tf_new);
v1_new = diff(q1_new);
acc1_new = diff(v1_new);

q2_new = (a10(2)+a11(2).*t_new+a12(2).*t_new.^2).*(t_new<=ts_new)+...
    (a20(2)+a21(2).*t_new).*(t_new>ts_new).*(t_new<tf_new-ts_new)+...
    (a30(2)+a31(2).*t_new+a32(2).*t_new.^2).*(t_new>=tf_new-ts_new).*(t_new<=tf_new);
v2_new = diff(q2_new);
acc2_new = diff(v2_new);

q3_new = (a10(3)+a11(3).*t_new+a12(3).*t_new.^2).*(t_new<=ts_new)+...
    (a20(3)+a21(3).*t_new).*(t_new>ts_new).*(t_new<tf_new-ts_new)+...
    (a30(3)+a31(3).*t_new+a32(3).*t_new.^2).*(t_new>=tf_new-ts_new).*(t_new<=tf_new);
v3_new = diff(q3_new);
acc3_new = diff(v3_new);

figure;
plot(t_new,q1_new,'b-',...
    t_new,q2_new,'k--',...
    t_new,q3_new,'r-.','linewidth',1);
xlabel('time, s');
ylabel('Position, rad');
title('Position vs time graph');
legend('Joint 1','Joint 2','Joint 3','Location','northwest');
axis([0 tf_new -inf +inf]);

figure;
plot(t_new(1:length(t_new)-1),v1_new,'b-',...
    t_new(1:length(t_new)-1),v2_new,'k--',...
    t_new(1:length(t_new)-1),v3_new,'r-.','linewidth',1);
xlabel('time, s');
ylabel('Velocity, rad/s');
title('Velocity vs time graph');
legend('Joint 1','Joint 2','Joint 3');
axis([0 tf_new -inf +inf]);

figure;
plot(t_new(1:length(t_new)-2),acc1_new,'b-',...
    t_new(1:length(t_new)-2),acc2_new,'k--',...
    t_new(1:length(t_new)-2),acc3_new,'r-.','linewidth',1);
xlabel('time, s');
ylabel('Acceleration, rad/s^2');
title('Acceleration vs time graph');
legend('Joint 1','Joint 2','Joint 3');
axis([0 tf_new -inf +inf]);