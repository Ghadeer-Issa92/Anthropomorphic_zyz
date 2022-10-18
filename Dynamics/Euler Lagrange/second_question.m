
clear all

%% load dynamics
load('M.mat')
load('C.mat')
load('G.mat')

%% load Trajectory
load('A.mat')
load('V.mat')
load('Q.mat')

%% Parameters values
l1=0.100;
l2=0.150;
l3=0.200;
l4=0.020;
l5=0.020;
l6=0.020;

lc1=l1/2;
lc2=l2/2;
lc3=l3/2;
lc4=l4/2;
lc5=l5/2;
lc6=l6/2;

m1=1; m2=1; m3=1; m4=1; m5=1; m6=1;
I1=eye(3); I2=eye(3); I3=eye(3); I4=eye(3); I5=eye(3); I6=eye(3);
g=9.8;

%% Calculate Torques

dt=1/20;
tau=zeros(6,length(Q));

for i=1:length(Q)
   q1=Q(1,i);q2=Q(2,i);q3=Q(3,i);q4=Q(4,i);q5=Q(5,i);q6=Q(6,i);
   dq1=V(1,i);dq2=V(2,i);dq3=V(3,i);dq4=V(4,i);dq5=V(5,i);dq6=V(6,i);
   ddq1=A(1,i);ddq2=A(2,i);q3=A(3,i);ddq4=A(4,i);ddq5=A(5,i);ddq6=A(6,i);

    MM=double(subs(M));
    B = MM(1:3:end,1:3:end);
    CC=double(subs(C));
    CD = CC(1:3:end,1:3:end);
    GG=double(subs(G));

tau(:,i)=B*A(:,i)+CD*V(:,i)+GG;
end

t=0:0.05:length(tau)*0.05-0.05;


figure('Name', 'Torques of first 3 joints');
axis(1) = subplot(3,1,1);
plot(t,tau(1,:));
xlabel('Time (s)');
ylabel('(N.m)');
title('Joint 1');
grid on
axis(2) = subplot(3,1,2);
plot(t,tau(2,:));
xlabel('Time (s)');
ylabel('(N.m)');
title('Joint 2');
grid on
axis(3) = subplot(3,1,3);
plot(t,tau(3,:));
xlabel('Time (s)');
ylabel('(N.m)');
title('Joint 3');
grid on

figure('Name', 'Torques of last 3 joints');
axis(1) = subplot(3,1,1);
plot(t,tau(4,:));
xlabel('Time (s)');
ylabel('(N.m)');
title('Joint 4');
grid on
axis(2) = subplot(3,1,2);
plot(t,tau(5,:));
xlabel('Time (s)');
ylabel('(N.m)');
title('Joint 5');
grid on
axis(3) = subplot(3,1,3);
plot(t,tau(6,:));
xlabel('Time (s)');
ylabel('(N.m)');
title('Joint 6');
grid on

save('tau.mat','tau')
