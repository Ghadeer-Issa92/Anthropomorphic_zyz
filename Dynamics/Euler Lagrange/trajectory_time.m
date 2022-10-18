function [t0,t1,T,tf]=trajectory_time(q0,qf,dq_m,ddq_m,t0)
  %q0,qf,dq_m,ddq_m = params

  dq = qf-q0;
  %##triangular check 
  c = sqrt(dq*ddq_m); 

  if c <= dq_m 
    t1 = sqrt(dq/ddq_m);
    T = t1;
    tf = 2*t1;
  else 
    t1 = dq_m/ddq_m;
    T = dq/dq_m;
    tf = T+t1; 
  end
end


