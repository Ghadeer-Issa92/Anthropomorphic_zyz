function [dw_i]=w_acceleration(joint_type,dw_i_1,w_i,R_i_1,dq_i,ddq_i,z_i_1)
if joint_type=='P' % prismatic joint
    dw_i=R_i_1'*dw_i_1;
end
if joint_type=='R' %  revolute joint
    dw_i=R_i_1'*(dw_i_1+ddq_i*z_i_1+cross(dq_i*w_i,z_i_1));
end
end