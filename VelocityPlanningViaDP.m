function [time, s, v, a] = VelocityPlanningViaDP()
global params_

NT = params_.dp.nt;
NS = params_.dp.ns;

% % Zero-layer node definition
zero_layer_node.cost = 0;
zero_layer_node.parent_id = [-999, -999];
zero_layer_node.cur_time = 0;
zero_layer_node.cur_s = 0.0;
zero_layer_node.parent_s = zero_layer_node.cur_s - params_.task.v0 * params_.dp.dt;
zero_layer_node.cur_v = params_.task.v0;
zero_layer_node.parent_v = params_.task.v0;
zero_layer_node.cur_a = params_.task.a0;

predefined_node = zero_layer_node;
predefined_node.cost = Inf;
state_space = repmat(predefined_node, NT, NS);

a = [];
v = [];
s = [];
cost = [];
% % Enumeration on the first layer
for jj = 1 : NS
    cur_node.cur_s = params_.dp.station_list(jj);
    cur_node.parent_s = zero_layer_node.cur_s;
    cur_node.parent_v = params_.task.v0;
    [cur_node.cur_a, cur_node.cur_v] = GetVAS(cur_node);
    cur_node.cur_time = zero_layer_node.cur_time + params_.dp.dt;
    cur_node.cost = CalculateCost(cur_node) + zero_layer_node.cost;
    cur_node.parent_id = [0, 0];
    state_space(1, jj) = cur_node;
    a = [a, cur_node.cur_a];
    v = [v, cur_node.cur_v];
    s = [s, cur_node.cur_s];
    cost = [cost, cur_node.cost];
end

for ii = 1 : (NT - 1)
    for jj = 1 : NS
        parent_node_candidate = state_space(ii, jj);
        for kk = 1 : NS
            cur_node.cur_s = params_.dp.station_list(kk);
            cur_node.parent_s = parent_node_candidate.cur_s;
            cur_node.parent_v = parent_node_candidate.cur_v;
            [cur_node.cur_a, cur_node.cur_v] = GetVAS(cur_node);
            cur_node.cur_time = parent_node_candidate.cur_time + params_.dp.dt;
            cur_node.cost = CalculateCost(cur_node) + parent_node_candidate.cost;
            obj = state_space(ii + 1, kk);
            if (cur_node.cost < obj.cost)
                cur_node.parent_id = [ii, jj]; % Change parent id
                state_space(ii + 1, kk) = cur_node;
            end
        end
    end
end

cur_best_cost = state_space(NT, 1).cost;
cur_best_s_ind = 1;
for ii = 2 : NS
    if (state_space(NT, ii).cost < cur_best_cost)
        cur_best_cost = state_space(NT, ii).cost;
        cur_best_s_ind = ii;
    end
end

disp('Cost derived by DP search = ');
disp(cur_best_cost);
if (cur_best_cost > 1e10)
    disp('Invalid DP search result.');
end

% % Node backtrack 1
s = [];
v = [];
a = [];
ptr = NT;
while (ptr > 0)
    elem = state_space(ptr, cur_best_s_ind);
    s = [elem.cur_s, s];
    v = [elem.cur_v, v];
    a = [elem.cur_a, a];
    cur_best_s_ind = elem.parent_id(2);
    ptr = ptr - 1;
end
s = [0, s];
v = [params_.task.v0, v];
a = [params_.task.a0, a];
time = linspace(0, params_.dp.time_horizon, length(s));
end