function [q,v,a,t]=plan_trajectory(q0,qf,dq_m,ddq_m,t0,t1,T,tf)

t=t0:0.05:tf-0.05;
q=zeros(1,length(t));
v=zeros(1,length(t));
a=zeros(1,length(t));

for i=1:length(t)
     if t(i) <=t1
        q(i) = q0 + (0.5*ddq_m*(t(i)-t0)^2);
        q02 = q(i);
        v(i) = ddq_m*t(i);
        v02 = v(i); 
        a(i) = ddq_m;
     elseif (t(i) > t1 && t(i) <= T)
             v(i) = dq_m;
            q(i) =  q02 + v02*(t(i)-t1);
            a(i) = 0;   
    else
        v(i) = ddq_m*(tf-t(i));
        q(i) = qf - (0.5*ddq_m*(t(i)-tf)^2);
        a(i) = -ddq_m;
     end
end

end

