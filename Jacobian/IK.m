function q=IK(T)
l1=0.4; l2=0.025; l3=0.56; l4=0.025; l5=0.515 ; l6=0.09;
Tn=T*inv(Tx(0.09));
%finf wrist center 
x=Tn(1,4); y=Tn(2,4); z=Tn(3,4);
if (x == 0 && y == 0)
    disp('singularity')
    q=NaN(2,6);
    return
end
q1 = atan2(y,x);
%q1=rad2deg(q1)
s = z - l1;
r = sqrt(x^2 + y^2)-l2;
A = (r^2 + s^2 - l5^2 - l4^2 - l3^2)/(2 * l3 * sqrt(l5^2 + l4^2));%cos(q3_)
if (abs(A) > 1)
    disp('Complex solution, Point unreachable');
    q=NaN(2,6);
    return
end
    q3_=atan2(sqrt(1-A^2),A);
    q3 = q3_ - atan2(l5,l4);
    %q3=rad2deg(q3)
    g = atan2(s,r);
    a = pi - q3_;
    B = (sin(a) * sqrt(l5^2 + l4^2))/sqrt(r^2 + s^2);
    b=atan2(B,sqrt(1-B^2));
    q2 = pi/2 - (g + b);
    %q2=rad2deg(q2)
    T123=Tz(0.4)*Rz(q1)*Tx(0.025)*Ry(q2)*Tz(0.56)*Ry(q3)*Tz(0.025)*Tx(0.515);
    T456=inv(T123)*T;
    T456=T456(1:3,1:3);
    T456(abs(T456)<1e-10)=0;
    if abs(T456(1,1))==1
        disp('singularity')
        q=NaN(2,6);
        return
    end
    %syms q4 q5 q6 real
    %R456=Rx(q4)*Ry(q5)*Rx(q6);
    %R456=simplify(R456);
    %R456=R456(1:3,1:3);
    q4=atan2(T456(2,1),-T456(3,1));
    %q4=rad2deg(q4);
    q6=atan2(T456(1,2),T456(1,3));
    %q6=rad2deg(q6);
    q5=atan2(sqrt(T456(1,3)^2+T456(1,2)^2),T456(1,1));
    %q5=rad2deg(q5);
    %Wrist second sol
    q41=atan2(-T456(2,1),T456(3,1));
    %q41=rad2deg(q41);
    q61=atan2(-T456(1,2),-T456(1,3));
    %q61=rad2deg(q61);
    q51=atan2(-sqrt(T456(1,3)^2+T456(1,2)^2),T456(1,1));
    %q51=rad2deg(q51);
    q = [q1 q2 q3 q4 q5 q6;
        q1 q2 q3 q41 q51 q61];
end