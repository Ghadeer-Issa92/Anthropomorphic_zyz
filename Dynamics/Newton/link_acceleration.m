function [ddp_i]=link_acceleration(joint_type,ddp_i_1,dw_i,w_i,R_i_1,dq_i,ddq_i,z_i_1,r_i_1)
if joint_type=='P' % prismatic joint
    ddp_i=R_i_1'*(ddp_i_1+ddq_i*z_i_1)+cross(2*dq_i*w_i,R_i_1'*z_i_1)+cross(dw_i,r_i_1)+cross(w_i,cross(w_i,r_i_1));
end
if joint_type=='R' % revolute  joint
    ddp_i=R_i_1'*ddp_i_1+cross(dw_i,r_i_1)+cross(w_i,cross(w_i,r_i_1));
end
end