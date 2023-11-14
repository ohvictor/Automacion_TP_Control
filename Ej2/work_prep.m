function [sim_q] = work_prep(t,A)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sim_q.time = t;
sim_q.signals.values = A;
end

