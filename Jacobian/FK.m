function [T,R03,R36]=FK(q)
T=Rz(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ry(q(3))*Tz(200)*Rz(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rz(q(6))*Tz(20);
T03=Rz(q(1))*Tz(100)*Ry(q(2))*Tz(150)*Ry(q(3))*Tz(200);
R03=T03(1:3,1:3);
R36=Rz(q(4))*Tz(20)*Ry(q(5))*Tz(20)*Rz(q(6))*Tz(20);
R36=R36(1:3,1:3);
end


