function J=Jn(q)
%Numerical derivatives 
disp('Numerical Method')
T=FK(q);
R=T(1:3,1:3);

j1=Rzd(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ry(q(3))*Tz(200)*Rz(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rz(q(6))*Tz(20)*[inv(R) zeros(3,1); 0 0 0 1];

J1=[j1(1,4);j1(2,4);j1(3,4);j1(3,2);j1(1,3);j1(2,1)];

j2=Rz(q(1))*Tz(100)*Ryd(q(2))*Tz(150)*Ry(q(3))*Tz(200)*Rz(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rz(q(6))*Tz(20)*[inv(R) zeros(3,1); 0 0 0 1];

J2=[j2(1,4);j2(2,4);j2(3,4);j2(3,2);j2(1,3);j2(2,1)];

j3=Rz(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ryd(q(3))*Tz(200)*Rz(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rz(q(6))*Tz(20)*[inv(R) zeros(3,1); 0 0 0 1];

J3=[j3(1,4);j3(2,4);j3(3,4);j3(3,2);j3(1,3);j3(2,1)];

j4=Rz(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ry(q(3))*Tz(200)*Rzd(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rz(q(6))*Tz(20)*[inv(R) zeros(3,1); 0 0 0 1];

J4=[j4(1,4);j4(2,4);j4(3,4);j4(3,2);j4(1,3);j4(2,1)];

j5=Rz(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ry(q(3))*Tz(200)*Rz(q(4))*Tz(20)*Ryd(q(5))*Tz(20)*Rz(q(6))*Tz(20)*[inv(R) zeros(3,1); 0 0 0 1];

J5=[j5(1,4);j5(2,4);j5(3,4);j5(3,2);j5(1,3);j5(2,1)];

j6=Rz(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ry(q(3))*Tz(200)*Rz(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rzd(q(6))*Tz(20)*[inv(R) zeros(3,1); 0 0 0 1];

J6=[j6(1,4);j6(2,4);j6(3,4);j6(3,2);j6(1,3);j6(2,1)];

J=[J1 J2 J3 J4 J5 J6];
end