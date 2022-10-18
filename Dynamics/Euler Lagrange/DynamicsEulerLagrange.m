
clear
clc



%Symbolics variable
syms q1 q2 q3 q4 q5 q6 real % robot joints
syms lc1 lc2 lc3 lc4 lc5 lc6 real % center of mass
syms l1 l2 l3 l4 l5 l6  real % links dimenstions
syms m1 m2 m3 m4 m5 m6 % links masses
syms I1 I2 I3 I4 I5 I6 % links Inertias
syms g % gravity



q = [q1, q2, q3, q4, q5, q6];
m = [m1, m2, m3, m4, m5, m6];
I = [I1, I2, I3, I4, I5, I6];
n = 6;
%% Mass or Inertia  matrix M

% First link
R0c1= HT('R','z',q1)*HT('T','z',lc1);
u0 = [0 0 1]';             

oc1  = simplify(R0c1(1:3,4));
Jv1 = [diff(oc1, q(1)) zeros(3,n-1)];
Jw1 = [u0 zeros(3,5)];


% Second link
R0c2  =HT('R','z',q1)*HT('T','z',l1)*HT('R','y',q2)*HT('T','z',lc2);
u1 = R0c1(1:3,2);              

oc2 = simplify(R0c2(1:3,4));
Jv2 = [diff(oc2, q(1)) diff(oc2, q(2)) zeros(3,n-2)];
Jv2 = simplify(Jv2);

Jw2 = [u0 u1 zeros(3,n-2)];
Jw2 = simplify(Jw2);


% Third link
R0c3  =HT('R','z',q1)*HT('T','z',l1)*HT('R','y',q2)*HT('T','z',l2)*HT('R','y',q3)*HT('T','z',lc3);
u2 = R0c2(1:3,2);            

oc3 = simplify(R0c3(1:3,4));

Jv3 = [diff(oc3, q(1)) diff(oc3, q(2)) diff(oc3, q(3)) zeros(3,n-3)];
Jv3 = simplify(Jv3);

Jw3 = [u0 u1 u2 zeros(3,n-3)];
Jw3 = simplify(Jw3);



% Fourth link
R0c4=HT('R','z',q1)*HT('T','z',l1)*HT('R','y',q2)*HT('T','z',l2)*HT('R','y',q3)*HT('T','z',l3)*HT('R','z',q4)*HT('T','z',lc4);
u3 = R0c3(1:3,3);             

oc4 = simplify(R0c4(1:3,4));
Jv4 = [diff(oc4, q(1)) diff(oc4, q(2)) diff(oc4, q(3)) diff(oc4, q(4)) zeros(3,n-4)];
Jv4 = simplify(Jv4);

Jw4 = [u0 u1 u2 u3 zeros(3,n-4)];
Jw4 = simplify(Jw4);

% Fifth link
R0c5=HT('R','z',q1)*HT('T','z',l1)*HT('R','y',q2)*HT('T','z',l2)*HT('R','y',q3)*HT('T','z',l3)*HT('R','z',q4)*HT('T','z',l4)*HT('R','y',q5)*HT('T','z',lc5);
u4 = R0c4(1:3,2);            

oc5 = simplify(R0c5(1:3,4));
Jv5 = [diff(oc5, q(1)) diff(oc5, q(2)) diff(oc5, q(3)) diff(oc5, q(4)) diff(oc5, q(5)) zeros(3,n-5)];
Jv5 = simplify(Jv5);

Jw5 = [u0 u1 u2 u3 u4 zeros(3,n-5)];
Jw5 = simplify(Jw5);


% Sixth link
R0c6=HT('R','z',q1)*HT('T','z',l1)*HT('R','y',q2)*HT('T','z',l2)*HT('R','y',q3)*HT('T','z',l3)*HT('R','z',q4)*HT('T','z',l4)*HT('R','y',q5)*HT('T','z',l5)*HT('R','z',q6)*HT('T','z',lc6);
u5 = R0c5(1:3,3);              

oc6 = simplify(R0c6(1:3,4));
Jv6 = [diff(oc6, q(1)) diff(oc6, q(2)) diff(oc6, q(3)) diff(oc6, q(4)) diff(oc6, q(5)) diff(oc6, q(6))];
Jv6 = simplify(Jv6);

Jw6 = [u0 u1 u2 u3 u4 u5];
Jw6 = simplify(Jw6);


Jv = [Jv1, Jv2, Jv3, Jv4, Jv5, Jv6];
Jw = [Jw1, Jw2, Jw3, Jw4, Jw5, Jw6];

R1 = R0c1(1:3,1:3); R2 = R0c2(1:3,1:3); R3 = R0c3(1:3,1:3);
R4 = R0c4(1:3,1:3); R5 = R0c5(1:3,1:3); R6 = R0c6(1:3,1:3);

R = [R1, R2, R3, R4, R5, R6];

syms M real
M1 =  m1 * (Jv1)' * Jv1 + (Jw1)' * R1 * I1 * R1' * Jw1; % link 1
M2 =  m2 * (Jv2)' * Jv2 + (Jw2)' * R2 * I2 * R2' * Jw2; % link 2
M3 =  m3 * (Jv3)' * Jv3 + (Jw3)' * R3 * I3 * R3' * Jw3; % link 3
M4 =  m4 * (Jv4)' * Jv4 + (Jw4)' * R4 * I4 * R4' * Jw4; % link 4
M5 =  m5 * (Jv5)' * Jv5 + (Jw5)' * R5 * I5 * R5' * Jw5; % link 5
M6 =  m6 * (Jv6)' * Jv6 + (Jw6)' * R6 * I6 * R6' * Jw6; % link 5

M = M1 + M2 + M3 + M4 + M5 + M6;


%% Coriolis Matrix C
C = coriolis(M,q);

%% Gravity  Matrix G
g0 = g*[0 0 -1]';

G1 = -1 * Jv1(:,1)' * m1 * g0 -1 * Jv2(:,1)' * m2 * g0 -1 * Jv3(:,1)' * m3 * g0 -1 * Jv4(:,1)' * m4 * g0 -1 * Jv5(:,1)' * m5 * g0 -1 * Jv6(:,1)' * m6 * g0;
G2 = -1 * Jv1(:,2)' * m1 * g0 -1 * Jv2(:,2)' * m2 * g0 -1 * Jv3(:,2)' * m3 * g0 -1 * Jv4(:,2)' * m4 * g0 -1 * Jv5(:,2)' * m5 * g0 -1 * Jv6(:,2)' * m6 * g0;
G3 = -1 * Jv1(:,3)' * m1 * g0 -1 * Jv2(:,3)' * m2 * g0 -1 * Jv3(:,3)' * m3 * g0 -1 * Jv4(:,3)' * m4 * g0 -1 * Jv5(:,3)' * m5 * g0 -1 * Jv6(:,3)' * m6 * g0;
G4 = -1 * Jv1(:,4)' * m1 * g0 -1 * Jv2(:,4)' * m2 * g0 -1 * Jv3(:,4)' * m3 * g0 -1 * Jv4(:,4)' * m4 * g0 -1 * Jv5(:,4)' * m5 * g0 -1 * Jv6(:,4)' * m6 * g0;
G5 = -1 * Jv1(:,5)' * m1 * g0 -1 * Jv2(:,5)' * m2 * g0 -1 * Jv3(:,5)' * m3 * g0 -1 * Jv4(:,5)' * m4 * g0 -1 * Jv5(:,5)' * m5 * g0 -1 * Jv6(:,5)' * m6 * g0;
G6 = -1 * Jv1(:,6)' * m1 * g0 -1 * Jv2(:,6)' * m2 * g0 -1 * Jv3(:,6)' * m3 * g0 -1 * Jv4(:,6)' * m4 * g0 -1 * Jv5(:,6)' * m5 * g0 -1 * Jv6(:,6)' * m6 * g0;

G = [G1; G2; G3; G4; G5; G6];
G = simplify(G);

%% save dynamics
save('M.mat','M')
save('C.mat','C')
save('G.mat','G')


