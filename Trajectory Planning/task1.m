%task1

[q1, v1, a1, t1, tb1, tc1, tf1] = calc_trajectory(0, 30, 5, 2);
[q2, v2, a2, t2, tb2, tc2, tf2] = calc_trajectory(20, 40, 5, 2);
[q3, v3, a3, t3, tb3, tc3, tf3] = calc_trajectory(40, 90, 7, 3);
[q4, v4, a4, t4, tb4, tc4, tf4] = calc_trajectory(0, 25, 7, 3);
[q5, v5, a5, t5, tb5, tc5, tf5] = calc_trajectory(5, 45, 7, 3);
[q6, v6, a6, t6, tb6, tc6, tf6] = calc_trajectory(10, 60, 7, 3);

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
