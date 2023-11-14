function [sim_q] = work_prep(t,A)
%work_prep Preparo las variables para ser inyectadas al modelo simulink
%   Detailed explanation goes here
sim_q.time = t;
sim_q.signals.values = A;
end

