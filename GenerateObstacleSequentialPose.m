function GenerateObstacleSequentialPose()
global params_
params_.dp.obstacle = [];
params_.obs_num_moving_obs = length(params_.dp.demo.obstacle_direction_angle);
time_sequence = 0 : params_.dp.unit_time_gap_for_resampling : params_.dp.time_horizon;

for ii = 1 : params_.obs_num_moving_obs
    cur_obs_pose = params_.dp.demo.obstacle_init_pose(ii, :);
    obs_moving_trajectory.x = cur_obs_pose(1) + time_sequence * params_.dp.demo.obstacle_speed(ii) * cos(params_.dp.demo.obstacle_direction_angle(ii));
    obs_moving_trajectory.y = cur_obs_pose(2) + time_sequence * params_.dp.demo.obstacle_speed(ii) * sin(params_.dp.demo.obstacle_direction_angle(ii));
    obs_moving_trajectory.theta = ones(1, length(time_sequence)) .* params_.dp.demo.obstacle_direction_angle(ii);
    params_.dp.obstacle = [params_.dp.obstacle, obs_moving_trajectory];
end
end