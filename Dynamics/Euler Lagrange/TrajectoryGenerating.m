clear all
clc


%% 1 

% position, velocity, and acceleration for 6 joints
q0_1=0;qf_1=pi;dq_max_1=0.1; ddq_max_1=0.1;
q0_2=0;qf_2=pi;dq_max_2=0.1; ddq_max_2=0.1;
q0_3=0;qf_3=pi;dq_max_3=0.1; ddq_max_3=0.1;
q0_4=0;qf_4=pi;dq_max_4=0.1; ddq_max_4=0.1;
q0_5=0;qf_5=pi;dq_max_5=0.1; ddq_max_5=0.1;
q0_6=0;qf_6=pi;dq_max_6=0.1; ddq_max_6=0.1;

t0=0; 

% calculate trajectory time for each joint
% trajectory_time function take intial and final position, max velocity and
% acceleration and return timing prameters (triangular or trapezoidal) tb,
% and dwelling T and tf
[t0_1,tb_1,T_1,tf_1]  = trajectory_time(q0_1,qf_1,dq_max_1, ddq_max_1,t0);
[t0_2,tb_2,T_2,tf_2]  = trajectory_time(q0_2,qf_2,dq_max_2, ddq_max_2,t0);
[t0_3,tb_3,T_3,tf_3]  = trajectory_time(q0_3,qf_3,dq_max_3, ddq_max_3,t0);
[t0_4,tb_4,T_4,tf_4]  = trajectory_time(q0_4,qf_4,dq_max_4, ddq_max_4,t0);
[t0_5,tb_5,T_5,tf_5]  = trajectory_time(q0_5,qf_5,dq_max_5, ddq_max_5,t0);
[t0_6,tb_6,T_6,tf_6]  = trajectory_time(q0_1,qf_6,dq_max_6, ddq_max_6,t0);


% calculate  trajectory for each joint
% plan_trajectory take all trajectory parameters and return position, velocity, and acceleration
[q1,v1,a1,t_1]= plan_trajectory(q0_1,qf_1,dq_max_1, ddq_max_1,t0_1,tb_1,T_1,tf_1);
[q2,v2,a2,t_2]= plan_trajectory(q0_2,qf_2,dq_max_2, ddq_max_2,t0_2,tb_2,T_2,tf_2);
[q3,v3,a3,t_3]= plan_trajectory(q0_3,qf_3,dq_max_3, ddq_max_3,t0_3,tb_3,T_3,tf_3);
[q4,v4,a4,t_4]= plan_trajectory(q0_4,qf_4,dq_max_4, ddq_max_4,t0_4,tb_4,T_4,tf_4);
[q5,v5,a5,t_5]= plan_trajectory(q0_5,qf_5,dq_max_5, ddq_max_5,t0_5,tb_5,T_5,tf_5);
[q6,v6,a6,t_6]= plan_trajectory(q0_6,qf_6,dq_max_6, ddq_max_6,t0_6,tb_6,T_6,tf_6);

% plot of position, velocity, and acceleration for 6 joints
figure('Name', 'Robot Joints');
axis(1) = subplot(3,1,1);
plot(t_1,q1);
hold on;
plot(t_2,q2,'r');
hold on;
plot(t_3,q3,'b');
hold on
plot(t_4,q4,'g');
hold on
plot(t_5,q5,'m');
hold on
plot(t_6,q6,'c');
hold on
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('(Degree)');
title('Postion');
hold off;
grid on
axis(2) = subplot(3,1,2);
plot(t_1,v1);
hold on;
plot(t_2,v2,'r');
hold on;
plot(t_3,v3,'b');
hold on
plot(t_4,v4,'g');
hold on
plot(t_5,v5,'m');
hold on
plot(t_6,v6,'c');
hold on
grid on
xlabel('Time (s)');
ylabel('(Degree/s)');
title('Velocity');
axis(3) = subplot(3,1,3);
plot(t_1,a1);
hold on;
plot(t_2,a2,'r');
hold on;
plot(t_3,a3,'b');
hold on
plot(t_4,a4,'g');
hold on
plot(t_5,a5,'m');
hold on
plot(t_6,a6,'c');
hold on
grid on
xlabel('Time (s)');
ylabel('(Degree/s^2)');
title('Acceleration');
hold off;

%% 
% 2) synchronization of 6 joints 

% synchronize according to the slowest joint
tb_s = max([tb_1,tb_2,tb_3,tb_4,tb_5,tb_6]);

dwell = max([T_1-tb_1, T_2-tb_2,T_3-tb_3,T_4-tb_4,T_5-tb_5,T_6-tb_6]);
T_s = dwell+tb_s;
tf_s = T_s + tb_s;

%recalculate velocity parameters
dq_max_1 = (qf_1-q0_1)/T_s;
dq_max_2 = (qf_2-q0_2)/T_s;
dq_max_3 = (qf_3-q0_3)/T_s;
dq_max_4 = (qf_4-q0_4)/T_s;
dq_max_5 = (qf_5-q0_5)/T_s;
dq_max_6 = (qf_6-q0_6)/T_s;

%recalculate acceleration parameters
ddq_max_1 = dq_max_1/tb_s;
ddq_max_2 = dq_max_2/tb_s;
ddq_max_3 = dq_max_3/tb_s;
ddq_max_4 = dq_max_4/tb_s;
ddq_max_5 = dq_max_5/tb_s;
ddq_max_6 = dq_max_6/tb_s;


%update parameters 
t0=0; 

% update trajectory time 
[t0_1,tb_1,T_1,tf_1]  = trajectory_time(q0_1,qf_1,dq_max_1, ddq_max_1,t0);
[t0_2,tb_2,T_2,tf_2]  = trajectory_time(q0_2,qf_2,dq_max_2, ddq_max_2,t0);
[t0_3,tb_3,T_3,tf_3]  = trajectory_time(q0_3,qf_3,dq_max_3, ddq_max_3,t0);
[t0_4,tb_4,T_4,tf_4]  = trajectory_time(q0_4,qf_4,dq_max_4, ddq_max_4,t0);
[t0_5,tb_5,T_5,tf_5]  = trajectory_time(q0_5,qf_5,dq_max_5, ddq_max_5,t0);
[t0_6,tb_6,T_6,tf_6]  = trajectory_time(q0_1,qf_6,dq_max_6, ddq_max_6,t0);


%update trajectory 
[q1,v1,a1,t_1]= plan_trajectory(q0_1,qf_1,dq_max_1, ddq_max_1,t0_1,tb_1,T_1,tf_1);
[q2,v2,a2,t_2]= plan_trajectory(q0_2,qf_2,dq_max_2, ddq_max_2,t0_2,tb_2,T_2,tf_2);
[q3,v3,a3,t_3]= plan_trajectory(q0_3,qf_3,dq_max_3, ddq_max_3,t0_3,tb_3,T_3,tf_3);
[q4,v4,a4,t_4]= plan_trajectory(q0_4,qf_4,dq_max_4, ddq_max_4,t0_4,tb_4,T_4,tf_4);
[q5,v5,a5,t_5]= plan_trajectory(q0_5,qf_5,dq_max_5, ddq_max_5,t0_5,tb_5,T_5,tf_5);
[q6,v6,a6,t_6]= plan_trajectory(q0_6,qf_6,dq_max_6, ddq_max_6,t0_6,tb_6,T_6,tf_6);


figure('Name', ' synchronization');
axis(1) = subplot(3,1,1);
plot(t_1,q1);
hold on;
plot(t_2,q2,'r');
hold on;
plot(t_3,q3,'b');
hold on
plot(t_4,q4,'g');
hold on
plot(t_5,q5,'m');
hold on
plot(t_6,q6,'c');
hold on
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('(Degree)');
title('Postion');
hold off;
grid on
axis(2) = subplot(3,1,2);
plot(t_1,v1);
hold on;
plot(t_2,v2,'r');
hold on;
plot(t_3,v3,'b');
hold on
plot(t_4,v4,'g');
hold on
plot(t_5,v5,'m');
hold on
plot(t_6,v6,'c');
hold on
grid on
xlabel('Time (s)');
ylabel('(Degree/s)');
title('Velocity');
axis(3) = subplot(3,1,3);
plot(t_1,a1);
hold on;
plot(t_2,a2,'r');
hold on;
plot(t_3,a3,'b');
hold on
plot(t_4,a4,'g');
hold on
plot(t_5,a5,'m');
hold on
plot(t_6,a6,'c');
hold on
grid on
xlabel('Time (s)');
ylabel('(Degree/s^2)');
title('Acceleration');
hold off;

%% numerical control 3 and 4
%controller frequency
f=20;
% movement for each joint
D_q_1 = qf_1-q0_1;
D_q_2 = qf_2-q0_2;
D_q_3 = qf_3-q0_3;
D_q_4 = qf_4-q0_4;
D_q_5 = qf_5-q0_5;
D_q_6 = qf_6-q0_6;

% numericalControl function calculates the error in joint position caused
% by controller frequency and recalculates the timing parameters tb and T and tf also
% recalculates max velocity and max acceleration to resynchronize the
% joints

[Delta_q1_error,dq_max_s_1,ddq_max_s_1,tb_s_1,T_s_1]=numericalControl(D_q_1,ddq_max_1,tb_1,T_1,f);
[Delta_q2_error,dq_max_s_2,ddq_max_s_2,tb_s_2,T_s_2]=numericalControl(D_q_2,ddq_max_2,tb_2,T_2,f);
[Delta_q3_error,dq_max_s_3,ddq_max_s_3,tb_s_3,T_s_3]=numericalControl(D_q_3,ddq_max_3,tb_3,T_3,f);
[Delta_q4_error,dq_max_s_4,ddq_max_s_4,tb_s_4,T_s_4]=numericalControl(D_q_4,ddq_max_4,tb_4,T_4,f);
[Delta_q5_error,dq_max_s_5,ddq_max_s_5,tb_s_5,T_s_5]=numericalControl(D_q_5,ddq_max_5,tb_5,T_5,f);
[Delta_q6_error,dq_max_s_6,ddq_max_s_6,tb_s_6,T_s_6]=numericalControl(D_q_6,ddq_max_6,tb_6,T_6,f);

% propagated error in end-effector position (question 3)
total_error=Delta_q1_error+Delta_q2_error+Delta_q3_error+Delta_q4_error+Delta_q5_error+Delta_q6_error;

% (question 4)
%update trajectory time 
[t0_1c,tb_1c,T_1c,tf_1c]  = trajectory_time(q0_1,qf_1,dq_max_s_1, ddq_max_s_1,t0);
[t0_2c,tb_2c,T_2c,tf_2c]  = trajectory_time(q0_2,qf_2,dq_max_s_2, ddq_max_s_2,t0);
[t0_3c,tb_3c,T_3c,tf_3c]  = trajectory_time(q0_3,qf_3,dq_max_s_3, ddq_max_s_3,t0);
[t0_4c,tb_4c,T_4c,tf_4c]  = trajectory_time(q0_4,qf_4,dq_max_s_4, ddq_max_s_4,t0);
[t0_5c,tb_5c,T_5c,tf_5c]  = trajectory_time(q0_5,qf_5,dq_max_s_5, ddq_max_s_5,t0);
[t0_6c,tb_6c,T_6c,tf_6c]  = trajectory_time(q0_1,qf_6,dq_max_s_6, ddq_max_s_6,t0);

%update trajectory 
[q1c,v1c,a1c,t_1c]= plan_trajectory(q0_1,qf_1,dq_max_1, ddq_max_1,t0_1c,tb_1c,T_1c,tf_1c);
[q2c,v2c,a2c,t_2c]= plan_trajectory(q0_2,qf_2,dq_max_2, ddq_max_2,t0_2c,tb_2c,T_2c,tf_2c);
[q3c,v3c,a3c,t_3c]= plan_trajectory(q0_3,qf_3,dq_max_3, ddq_max_3,t0_3c,tb_3c,T_3c,tf_3c);
[q4c,v4c,a4c,t_4c]= plan_trajectory(q0_4,qf_4,dq_max_4, ddq_max_4,t0_4c,tb_4c,T_4c,tf_4c);
[q5c,v5c,a5c,t_5c]= plan_trajectory(q0_5,qf_5,dq_max_5, ddq_max_5,t0_5c,tb_5c,T_5c,tf_5c);
[q6c,v6c,a6c,t_6c]= plan_trajectory(q0_6,qf_6,dq_max_6, ddq_max_6,t0_6c,tb_6c,T_6c,tf_6c);


figure('Name', ' synchronization with numerical control');
axis(1) = subplot(3,1,1);
plot(t_1c,q1c);
hold on;
plot(t_2c,q2c,'r');
hold on;
plot(t_3c,q3c,'b');
hold on
plot(t_4c,q4c,'g');
hold on
plot(t_5c,q5c,'m');
hold on
plot(t_6c,q6c,'c');
hold on
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('(Degree)');
title('Postion');
hold off;
grid on
axis(2) = subplot(3,1,2);
plot(t_1c,v1c);
hold on;
plot(t_2c,v2c,'r');
hold on;
plot(t_3c,v3c,'b');
hold on
plot(t_4c,v4c,'g');
hold on
plot(t_5c,v5c,'m');
hold on
plot(t_6c,v6c,'c');
hold on
grid on
xlabel('Time (s)');
ylabel('(Degree/s)');
title('Velocity');
axis(3) = subplot(3,1,3);
plot(t_1c,a1c);
hold on;
plot(t_2c,a2c,'r');
hold on;
plot(t_3c,a3c,'b');
hold on
plot(t_4c,a4c,'g');
hold on
plot(t_5c,a5c,'m');
hold on
plot(t_6c,a6c,'c');
hold on
grid on
xlabel('Time (s)');
ylabel('(Degree/s^2)');
title('Acceleration');
hold off;

Q=[q1c;q2c;q3c;q4c;q5c;q6c];
V=[v1c;v2c;v3c;v4c;v5c;v6c];
A=[a1c;a2c;a3c;a4c;a5c;a6c];

save('A.mat','A')
save('V.mat','V')
save('Q.mat','Q')
