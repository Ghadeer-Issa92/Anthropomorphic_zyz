%task3

q0 = [0, 20, 40, 0, 5, 10];
qf = [30, 40, 90, 25, 45, 60];


[~, ~, ~, ~, tb1, ~, tf1] = calc_trajectory(0, 30, 5, 2);
[~, ~, ~, ~, tb2, ~, tf2] = calc_trajectory(20, 40, 5, 2);
[~, ~, ~, ~, tb3, ~, tf3] = calc_trajectory(40, 90, 7, 3);
[~, ~, ~, ~, tb4, ~, tf4] = calc_trajectory(0, 25, 7, 3);
[~, ~, ~, ~, tb5, ~, tf5] = calc_trajectory(5, 45, 7, 3);
[~, ~, ~, ~, tb6, ~, tf6] = calc_trajectory(10, 60, 7, 3);

%find max time 
tf_new = max([tf1, tf2, tf3, tf4, tf5, tf6]);
tb_new = max([tb1, tb2, tb3, tb4, tb5, tb6]);

%recalculate v_max and a_max
v_max1 = (qf(1) - q0(1))/tf_new;
a_max1 = v_max1/tb_new;

v_max2 = (qf(2) - q0(2))/tf_new;
a_max2 = v_max2/tb_new;

v_max3 = (qf(3) - q0(3))/tf_new;
a_max3 = v_max3/tb_new;

v_max4 = (qf(4) - q0(4))/tf_new;
a_max4 = v_max4/tb_new;

v_max5 = (qf(5) - q0(5))/tf_new;
a_max5 = v_max5/tb_new;

v_max6 = (qf(6) - q0(6))/tf_new;
a_max6 = v_max6/tb_new;


[~, ~, ~, ~, tb1, tc1, ~] = calc_trajectory(0, 30, v_max1, a_max1);
[~, ~, ~, ~, tb2, tc2, ~] = calc_trajectory(20, 40, v_max2, a_max2);
[~, ~, ~, ~, tb3, tc3, ~] = calc_trajectory(40, 90, v_max3, a_max3);
[~, ~, ~, ~, tb4, tc4, ~] = calc_trajectory(0, 25, v_max4, a_max4);
[~, ~, ~, ~, tb5, tc5, ~] = calc_trajectory(5, 45, v_max5, a_max5);
[~, ~, ~, ~, tb6, tc6, ~] = calc_trajectory(10, 60, v_max6, a_max6);


[q1, v1, a1, t1, Dq1] = controller_sync(0, 30, a_max1, tb1, tc1);
[q2, v2, a2, t2, Dq2] = controller_sync(20, 40, a_max2, tb2, tc2);
[q3, v3, a3, t3, Dq3] = controller_sync(40, 90, a_max3, tb3, tc3);
[q4, v4, a4, t4, Dq4] = controller_sync(0, 25, a_max4, tb4, tc4);
[q5, v5, a5, t5, Dq5] = controller_sync(5, 45, a_max5, tb5, tc5);
[q6, v6, a6, t6, Dq6] = controller_sync(0, 30, a_max6, tb6, tc6);

total_error = Dq1 + Dq2 +Dq3 +Dq4 +Dq5 + Dq5;

figure('Name', 'position velocity acceleration');
axis(1) = subplot(3,1,1);
plot(t1,q1);
hold on;
plot(t2,q2,'r');
hold on;
plot(t3,q3,'b');
hold on
plot(t4,q4,'g');
hold on
plot(t5,q5,'y');
hold on
plot(t6,q6,'k');
hold on
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time');
ylabel('Degree');
title('Postion');
hold off;
grid on
axis(2) = subplot(3,1,2);
plot(t1,v1);
hold on;
plot(t2,v2,'r');
hold on;
plot(t3,v3,'b');
hold on
plot(t4,v4,'g');
hold on
plot(t5,v5,'y');
hold on
plot(t6,v6,'k');
hold on
grid on
xlabel('Time');
ylabel('Degree/s');
title('Velocity');
axis(3) = subplot(3,1,3);
plot(t1,a1);
hold on;
plot(t2,a2,'r');
hold on;
plot(t3,a3,'b');
hold on
plot(t4,a4,'g');
hold on
plot(t5,a5,'y');
hold on
plot(t6,a6,'k');
hold on
grid on
xlabel('Time');
ylabel('Degree/s^2');
title('Acceleration');
hold off;
