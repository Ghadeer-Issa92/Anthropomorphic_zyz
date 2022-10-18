
load('tau.mat')
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