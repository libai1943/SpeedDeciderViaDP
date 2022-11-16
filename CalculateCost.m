function cost = CalculateCost(cur_node)
global params_
%% Penalize excessive velocity
if ((cur_node.cur_v < 0)||(cur_node.cur_v > params_.dp.v_max))
    cost = 1e20;
    return;
end
%% Penalize excessive acceleration
if ((cur_node.cur_a < params_.dp.a_min)||(cur_node.cur_a > params_.dp.a_max))
    cost = 1e20;
    return;
end

%% Penalize on collisions
if (IsCurNodeCollidedToObs(cur_node))
    cost = 1e20;
    return;
end

cost = 0;
%% Penalize non-zero acceleration
cost_a = (cur_node.cur_a^2) / (1 + exp(cur_node.cur_a - params_.dp.a_min)) + ...
    (cur_node.cur_a^2) / (1 + exp(params_.dp.a_max - cur_node.cur_a));
cost = cost + params_.dp.weight.acc * cost_a;

%% Penalize bias from nominal velocity
cost_v = (cur_node.cur_v - params_.dp.nominal_v_cruising)^2;
cost = cost + params_.dp.weight.norm_speed * cost_v;
end