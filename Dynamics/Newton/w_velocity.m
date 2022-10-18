function [w_i]=w_velocity(joint_type,w_i_1,R_i_1,dq_i,z_i_1)
if joint_type=='P' % prismatic  joint
    w_i=R_i_1'*w_i_1;
end
if joint_type=='R' % revolute joint
        w_i=R_i_1'*(w_i_1+dq_i*z_i_1);
end
end