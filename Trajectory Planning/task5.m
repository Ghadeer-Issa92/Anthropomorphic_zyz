%for this task I am printing plots only for the first three joints, while
%for the last three joints I am just calculating the coefs while plotting
%is absolutly possible but I think it does not serve any important purpose
%in this case.

q10 = 0; q1f = 30; v10 = 0; v1f = 0;acc10 = 0;acc1f = 0;
q20 = 0; q2f = 35;  v20 = 0; v2f = 0; acc20 = 0;  acc2f = 0;
q30 = 0; q3f = 50;v30=0; v3f=0; acc30=0; acc3f=0;
q40 = 0; q4f = 30;v40=0; v4f=0; acc40=0; acc4f=0;
q50 = 0; q5f = 30;v50=0; v5f=0; acc50=0; acc5f=0;
q60 = 0; q6f = 25;v60=0; v6f=0; acc60=0; acc6f=0;
t0=0;tf=2;
A = [1 t0 t0^2 t0^3 t0^4 t0^5
     0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
     0 0 2 6*t0 12*t0^2 20*t0^3
     1 tf tf^2 tf^3 tf^4 tf^5
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
     0 0 2 6*tf 12*tf^2 20*tf^3];
c1 = [q10;v10;acc10;q1f;v1f;acc1f];
b1 = A\c1;
% computing for joint 1
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);
t = t0:0.01:tf;
 q1 = a01+a11.*t+a21.*t.^2+a31.*t.^3+a41.*t.^4+a51.*t.^5;
 v1 = a11+2*a21.*t+3*a31.*t.^2+4*a41.*t.^3+5*a51.*t.^4;
 acc1 = 2*a21+6*a31.*t+12*a41.*t.^2+20*a51.*t.^3;
 subplot(3,3,1)
 plot(t,q1,'g-')
 title('joint1 position vs time')
 grid on
 subplot(3,3,2)
 plot(t,v1,'b-')
 title('joint1 velocity vs time')
 grid on
  subplot(3,3,3)
 plot(t,acc1,'k-')
 title('joint1 acceleration vs time')
 grid on
 % computing for joint 2
c1 = [q20;v20;acc20;q2f;v2f;acc2f];
b1 = A\c1;
% computing for joint 2
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);
t = t0:0.01:tf;
 q2 = a01+a11.*t+a21.*t.^2+a31.*t.^3+a41.*t.^4+a51.*t.^5;
 v2 = a11+2*a21.*t+3*a31.*t.^2+4*a41.*t.^3+5*a51.*t.^4;
 acc2 = 2*a21+6*a31.*t+12*a41.*t.^2+20*a51.*t.^3;
  subplot(3,3,4)
 plot(t,q2,'g-')
 title('joint2 position vs time')
 grid on
 subplot(3,3,5)
 plot(t,v2,'b-')
 title('joint2 velocity vs time')
 grid on
 subplot(3,3,6)
 plot(t,acc2,'k-')
 title('joint2 acceleration vs time')
 grid on
 % computing for joint 3
 c1 = [q30;v30;acc30;q3f;v3f;acc3f];
b1 = A\c1;
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);
t = t0:0.01:tf;
 q3 = a01+a11.*t+a21.*t.^2+a31.*t.^3+a41.*t.^4+a51.*t.^5;
 v3 = a11+2*a21.*t+3*a31.*t.^2+4*a41.*t.^3+5*a51.*t.^4;
 acc3 = 2*a21+6*a31.*t+12*a41.*t.^2+20*a51.*t.^3;
 subplot(3,3,7)
 plot(t,q3,'g-')
 title('joint3 position vs time')
 grid on
 subplot(3,3,8)
 plot(t,v3,'b-')
 title('joint3 velocity vs time')
 grid on
 subplot(3,3,9)
 plot(t,acc3,'k-')
 title('joint3 acceleration vs time')
 grid on
 
% computing for joint 4
c1 = [q40;v40;acc40;q4f;v4f;acc4f];
b1 = A\c1;
a01 = b1(1); a11 = b1(2); a21 = b1(3); a31 = b1(4); a41 = b1(5); a51 = b1(6);

% computing for joint 5
c1 = [q50;v50;acc50;q5f;v5f;acc5f];
b1 = A\c1;
u01 = b1(1); u11 = b1(2); u21 = b1(3); u31 = b1(4); u41 = b1(5); u51 = b1(6);


% computing for joint 6
c1 = [q60;v60;acc60;q6f;v6f;acc6f];
b1 = A\c1;
y01 = b1(1); y11 = b1(2); y21 = b1(3); y31 = b1(4); y41 = b1(5); y51 = b1(6);




 