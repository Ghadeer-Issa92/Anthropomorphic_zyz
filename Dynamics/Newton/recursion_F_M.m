function [f_i,M_i]=recursion_F_M(f_i1,R_i1,M_i1,m_i,ddpc_i,I_i,w_i,dw_i,r_i_1,rc_i)
        f_i=R_i1*f_i1+m_i*ddpc_i;
        M_i=R_i1*M_i1-cross(f_i,r_i_1+rc_i)+cross(R_i1*f_i1,rc_i)+I_i*dw_i+cross(w_i,I_i*w_i);
end