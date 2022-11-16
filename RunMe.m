clear all; close all; clc;
global params_

InitializeParams();
params_.task.v0 = 0;
params_.task.a0 = 0;

[time, s, v, a] = VelocityPlanningViaDP();
params_.s = s;
params_.v = v;
params_.a = a;
params_.time = time;

% % Plot dynamic process
asd();

% % Plot searched profiles
dsa();