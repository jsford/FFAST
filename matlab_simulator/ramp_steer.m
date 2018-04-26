clear all
close all
clc

global vehicle
load('vehicle.mat')

vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_traj_r = 1;
vis.rax_color = 'r';
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.titles = {'v_x','v_y','yaw rate'};

plt = init_plot(plt);
vis = init_vis(vis);

X = [0;0;0;0;0;0];
dt = 0.02;
cmd_vel = 1.5;

for steer = -30:0
    a = tic;
    U = [cmd_vel; degtorad(steer)];
    
    %t = tic;
    while (toc(a) < 1)
        vis = update_vis(vis, X, U);
        %dt = toc(t); t = tic;
        x = state_transition(X, U, dt);
        pause(dt)
        
        plt.vals = [x(4),x(5),x(6)];
        plt = update_plot(plt);
    end
end