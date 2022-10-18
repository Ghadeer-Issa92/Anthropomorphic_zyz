
clear
clc


%%%%
%%
%%
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
   q_1=Q(1,i);   q_2=Q(2,i);   q_3=Q(3,i);   q_4=Q(4,i);   q_5=Q(5,i);   q_6=Q(6,i);
   dq_1=V(1,i);  dq_2=V(2,i);  dq_3=V(3,i);  dq_4=V(4,i);  dq_5=V(5,i);  dq_6=V(6,i);
   ddq_1=A(1,i); ddq_2=A(2,i); ddq_3=A(3,i); ddq_4=A(4,i); ddq_5=A(5,i); ddq_6=A(6,i);


ddp_0=[0;0;g];
w_0_0=[0;0;0];dw_0_0=[0;0;0];
z_0=[0;0;1];
r_c_1=[0;0;lc1];
r_0_1=[0;0;l1];
R_0_1=HT('R','z',q_1);

% forward 

% link1
w_1_1=w_velocity('R',w_0_0,R_0_1,dq_1,z_0);
dw_1_1=w_acceleration('R',dw_0_0,w_0_0,R_0_1,dq_1,ddq_1,z_0);
ddp_1=link_acceleration('R',ddp_0,dw_1_1,w_1_1,R_0_1,dq_1,ddq_1,z_0,r_0_1);
ddpc_1=com_acceleration(ddp_1,dw_1_1,w_1_1,r_c_1);

% link2
z_1=[0;1;0];
r_c_2=[0;0;lc2];
r_1_2=[0;0;l2];
R_1_2=HT('R','y',q_2);

w_2_2=w_velocity('R',w_1_1,R_1_2,dq_2,z_1);
dw_2_2=w_acceleration('R',dw_1_1,w_1_1,R_1_2,dq_2,ddq_2,z_1);
ddp_2=link_acceleration('R',ddp_1,dw_2_2,w_2_2,R_1_2,dq_2,ddq_2,z_1,r_1_2);
ddpc_2=com_acceleration(ddp_2,dw_2_2,w_2_2,r_c_2);


% link 3
z_2=[0;1;0];
r_c_3=[0;0;lc3];
r_2_3=[0;0;l3];
R_2_3=HT('R','y',q_3);

w_3_3=w_velocity('R',w_2_2,R_2_3,dq_3,z_2);
dw_3_3=w_acceleration('R',dw_2_2,w_2_2,R_2_3,dq_3,ddq_3,z_2);
ddp_3=link_acceleration('R',ddp_2,dw_3_3,w_3_3,R_2_3,dq_3,ddq_3,z_2,r_2_3);
ddpc_3=com_acceleration(ddp_3,dw_3_3,w_3_3,r_c_3);


% link 4
z_3=[0;0;1];
r_c_4=[0;0;lc4];
r_3_4=[0;0;l4];
R_3_4=HT('R','z',q_4);

w_4_4=w_velocity('R',w_3_3,R_3_4,dq_4,z_3);
dw_4_4=w_acceleration('R',dw_3_3,w_3_3,R_3_4,dq_4,ddq_4,z_3);
ddp_4=link_acceleration('R',ddp_3,dw_4_4,w_4_4,R_3_4,dq_4,ddq_4,z_3,r_3_4);
ddpc_4=com_acceleration(ddp_4,dw_4_4,w_4_4,r_c_4);



% link 5
z_4=[0;1;0];
r_c_5=[0;0;lc5];
r_4_5=[0;0;l5];
R_4_5=HT('R','y',q_5);

w_5_5=w_velocity('R',w_4_4,R_4_5,dq_5,z_4);
dw_5_5=w_acceleration('R',dw_4_4,w_4_4,R_4_5,dq_5,ddq_5,z_4);
ddp_5=link_acceleration('R',ddp_4,dw_5_5,w_5_5,R_4_5,dq_5,ddq_5,z_4,r_4_5);
ddpc_5=com_acceleration(ddp_5,dw_5_5,w_5_5,r_c_5);

% link 6
z_5=[0;0;1];
r_c_6=[0;0;lc6];
r_5_6=[0;0;l6];
R_5_6=HT('R','z',q_6);

w_6_6=w_velocity('R',w_5_5,R_5_6,dq_6,z_5);
dw_6_6=w_acceleration('R',dw_5_5,w_5_5,R_5_6,dq_6,ddq_6,z_5);
ddp_6=link_acceleration('R',ddp_5,dw_6_6,w_6_6,R_5_6,dq_6,ddq_6,z_5,r_5_6);
ddpc_6=com_acceleration(ddp_6,dw_6_6,w_6_6,r_c_6);

% backward

%function [f_i,M_i]=recursion_F_M(f_i1,R_i1,M_i1,m_i,ddpc_i,I_i,w_i,dw_i,r_i_1,rc_i)
f_7=[0;0;0];M_7=[0;0;0];
R_6_7=eye(3);

[f_6,M_6]=recursion_F_M(f_7,R_6_7,M_7,m6,ddpc_6,I6,w_6_6,dw_6_6,r_5_6,r_c_6);
tau(6,i)=joint_torque('R',f_6,M_6,R_5_6,z_5);


[f_5,M_5]=recursion_F_M(f_6,R_5_6,M_6,m5,ddpc_5,I5,w_5_5,dw_5_5,r_4_5,r_c_5);
tau(5,i)=joint_torque('R',f_5,M_5,R_4_5,z_4);


[f_4,M_4]=recursion_F_M(f_5,R_4_5,M_5,m4,ddpc_4,I4,w_4_4,dw_4_4,r_3_4,r_c_4);
tau(4,i)=joint_torque('R',f_4,M_4,R_3_4,z_3);

[f_3,M_3]=recursion_F_M(f_4,R_3_4,M_4,m3,ddpc_3,I3,w_3_3,dw_3_3,r_2_3,r_c_3);
tau(3,i)=joint_torque('R',f_3,M_3,R_2_3,z_2);


[f_2,M_2]=recursion_F_M(f_3,R_2_3,M_3,m2,ddpc_2,I2,w_2_2,dw_2_2,r_1_2,r_c_2);
tau(2,i)=joint_torque('R',f_2,M_2,R_1_2,z_1);


[f_1,M_1]=recursion_F_M(f_2,R_1_2,M_2,m1,ddpc_1,I1,w_1_1,dw_1_1,r_0_1,r_c_1);
tau(1,i)=joint_torque('R',f_1,M_1,R_0_1,z_0);

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

%save('tau.mat','tau')
