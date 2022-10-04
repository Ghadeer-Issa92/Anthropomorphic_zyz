function [q, v, a, t, tb, tc, tf] = calc_trajectory(q0, qf, v_m, a_m)
    dq = qf - q0;
    flag = sqrt(dq * a_m);
    %check for traingular or trapezoidal profile and calculate time intervals 
    if flag <= v_m
        tb = sqrt(dq/a_m);
        tc = tb;
        tf = 2*tb;
    else
        tb = v_m/a_m;
        tc = dq/v_m;
        tf = tc+tb;
    end
    
    t=0:0.1:tf;
    q = zeros(1,length(t));
    v = zeros(1,length(t));
    a = zeros(1,length(t));
    
    %calculate q, v, a, t depending on the time interval 
    for i=1:length(t)
        if t(i) <=tb
            q(i) = q0 + (0.5*a_m*t(i)^2);
            q02 = q(i);
            v(i) = a_m*t(i);
            v02 = v(i); 
            a(i) = a_m;
        elseif (t(i) > tb && t(i) <= tc)
            v(i) = v_m;
            q(i) =  q02 + v02*(t(i)-tb);
            a(i) = 0;   
        else
            v(i) = a_m*(tf-t(i));
            q(i) = qf - (0.5*a_m*(t(i)-tf)^2);
            a(i) = -a_m;
        end
    end
end