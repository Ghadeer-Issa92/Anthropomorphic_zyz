function [tau_i]=joint_torque(joint_type,f_i,M_i,R_i_1,z_i_1)
if joint_type=='P' % prismatic joint
    tau_i=f_i'*R_i_1'*z_i_1;
end
if joint_type=='R' % revolute  joint
    tau_i=M_i'*R_i_1'*z_i_1;
end
end