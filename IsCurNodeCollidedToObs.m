function is_collided = IsCurNodeCollidedToObs(cur_node)
global params_
is_collided = 1;

distance = cur_node.cur_s - cur_node.parent_s;
if (distance < 0)
    error('Negative s1 - s0');
end
nfe = 2 + ceil(distance / params_.dp.unit_s_for_resampling);
timelist = linspace(cur_node.cur_time - params_.dp.dt, cur_node.cur_time, nfe);
slist = linspace(cur_node.parent_s, cur_node.cur_s, nfe);

for ii = 1 : nfe
    cur_subtle_time = timelist(ii);
    cur_subtle_s = slist(ii);
    V_ego_vehicle = CreateVehiclePolygon(cur_subtle_s, 0, 0, 2);

    id_in_obs = max(1, round(cur_subtle_time / params_.dp.unit_time_gap_for_resampling));

    obs_x = [];
    obs_y = [];
    for jj = 1 : params_.obs_num_moving_obs
        obs = params_.dp.obstacle(jj);
        V_moving_obs = CreateVehiclePolygon(obs.x(id_in_obs), obs.y(id_in_obs), obs.theta(id_in_obs), 30);
        obs_x = [obs_x, V_moving_obs.x];
        obs_y = [obs_y, V_moving_obs.y];
    end
    if (any(inpolygon(obs_x, obs_y, V_ego_vehicle.x, V_ego_vehicle.y)))
        return;
    end
end
is_collided = 0;
end