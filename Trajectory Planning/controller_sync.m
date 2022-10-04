function [q, v_traj, a_traj, t, Dq] = controller_sync(q0, qf, a_m, tb, tc)
dt = 1/20;
dq = qf - q0;
if rem(tb, dt) == 0
    control_tb = floor(tb/dt) * dt;
else
    control_tb = floor(tb/dt)*dt + dt;
end

if rem(tc, dt) == 0
    control_tc = floor(tc/dt) *dt;
else
    control_tc = floor(tc/dt) *dt + dt;
end

control_v = a_m * control_tb;
control_d = control_tc * control_v;
Dq = control_d - dq;

tb_sync = ceil(tb/dt) * dt;
tc_sync = ceil(tc/dt) * dt;
v_m_sync = dq/tc_sync;
a_m_sync = v_m_sync/tb_sync;

[q, v_traj, a_traj, t, tb_traj, ~, ~] = calc_trajectory(q0, qf, v_m_sync, a_m_sync);

display(tb_traj)
display(tb_sync)

