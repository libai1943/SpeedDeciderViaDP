function [a1, v1] = GetVAS(cur_node)
global params_
s1 = cur_node.cur_s;
s0 = cur_node.parent_s;
v0 = cur_node.parent_v;

v1 = (s1 - s0) / params_.dp.dt;
a1 = (v1 - v0) / params_.dp.dt;
end