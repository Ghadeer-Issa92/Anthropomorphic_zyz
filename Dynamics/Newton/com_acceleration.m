function [ddpc_i]=com_acceleration(ddp_i,dw_i,w_i,rc_i)

ddpc_i=ddp_i+cross(dw_i,rc_i)+cross(w_i,cross(w_i,rc_i));

end