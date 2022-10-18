function[Delta_q,dq_max_s,ddq_max_s,tb_s,T_s]=numericalControl(Dq,ddq_max,tb,T,f)
Delta_t=1/f;
% t1_1,T_1,tf_1
if rem(tb,Delta_t)==0
    real_tb=floor(tb/Delta_t)*Delta_t;
else
    real_tb=floor(tb/Delta_t)*Delta_t+Delta_t;
end

%dq_m_1, ddq_m_1
real_dq=ddq_max*real_tb;

if rem(T,Delta_t)==0
    real_T=floor(T/Delta_t)*Delta_t;
else
    real_T=floor(T/Delta_t)*Delta_t+Delta_t;
end

D_q_new=real_T*real_dq;
Delta_q=D_q_new-Dq;

tb_s=ceil(tb/Delta_t)*Delta_t;
T_s=ceil(T/Delta_t)*Delta_t;
dq_max_s = Dq/T_s;
ddq_max_s = dq_max_s/tb_s;


end