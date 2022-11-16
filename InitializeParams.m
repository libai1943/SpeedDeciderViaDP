function InitializeParams()
global params_

params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;

params_.dp.v_max = 10.0; % maximum speed is set to 10m/s
params_.dp.a_min = -5.0; % minimum acceleration is set to -5m/s^2
params_.dp.a_max = 3.0; % maximum acceleration is set to 3m/s^2

params_.dp.time_horizon = 10.0; % T_max is set to 10 seconds
params_.dp.s_horizon = params_.dp.time_horizon * params_.dp.v_max * 1.1; % Ensure s_goal is sufficiently large, 1.1 is a buffer
params_.dp.nominal_v_cruising = params_.dp.v_max * 0.8; % safe cruise speed on the road without obstacles
params_.dp.unit_time_gap_for_resampling = 0.01;
params_.dp.unit_s_for_resampling = 0.05;
params_.dp.num_units_for_resampling_between_adjacent_layers = 10;

params_.dp.nt = 15; % number of layers
params_.dp.ns = 100; % number of nodes in each layer
% Herein, nt means Number of Time layers while na means Number of Stations nodes in each layer.

params_.dp.dt = params_.dp.time_horizon / params_.dp.nt; % time gap between adjacent layers.
params_.dp.station_list = linspace(0.0, params_.dp.s_horizon, params_.dp.ns);

params_.dp.weight.acc = 1.0;
params_.dp.weight.norm_speed = 1.0;


params_.dp.demo.obstacle_speed = [3.0, 2.0, 0.1];
params_.dp.demo.obstacle_direction_angle = [0.2, 0.5 * pi, -0.5 * pi];
params_.dp.demo.obstacle_init_pose = [12.0, -2.0; 7, -2; 32, -1.1];

GenerateObstacleSequentialPose();
end