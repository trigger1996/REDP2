clear all;
close all;
clc;
 
bag = rosbag('D:\User\Videos\2024.3.Drone\2024-03-15-17-03-23.bag');
 

% Load Data from ROSBAG
%%
occupancy_grid_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/map');
occupancy_grid_msg = readMessages(occupancy_grid_select);
map = occupancy_grid_msg(size(occupancy_grid_msg, 1), 1);
map = map{1, 1};

map_width  = map.Info.Width;
map_height = map.Info.Height;
map_resolution = map.Info.Resolution;
map_2D = zeros(map_height,map_width);
map_x_offset = map.Info.Origin.Position.X;
map_y_offset = map.Info.Origin.Position.Y;

for i = 1 : map_width - 1
    for j = 1 : map_height
        %map_2D(j, map_width - i) = map.Data(i * map_width + j);
        map_2D(j, i) = map.Data(i * map_width + j);
    end
end

%%
laser_pose_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/poseupdate');
laser_pose_msg = readMessages(laser_pose_select);

laser_t_start = 0;
for i = 1 : size(laser_pose_msg, 1)
    laser_pose_t = laser_pose_msg(i, 1);
    laser_pose_t = laser_pose_t{1, 1};
    
    if i == 1
        laser_t_start = laser_pose_t.Header.Stamp.Sec + laser_pose_t.Header.Stamp.Nsec / 1.e9;
    end
    laser_t(i, :) = laser_pose_t.Header.Stamp.Sec + laser_pose_t.Header.Stamp.Nsec / 1.e9;

    laser_x(i, :) = laser_pose_t.Pose.Pose.Position.X;
    laser_y(i, :) = laser_pose_t.Pose.Pose.Position.Y;

    [laser_roll(i, :), laser_pitch(i, :), laser_yaw(i, :)] = quat2angle([laser_pose_t.Pose.Pose.Orientation.X, laser_pose_t.Pose.Pose.Orientation.Y, laser_pose_t.Pose.Pose.Orientation.Z, laser_pose_t.Pose.Pose.Orientation.W]);
end

%%
odom_pose_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/odom');
odom_pose_msg = readMessages(odom_pose_select);

odom_t_start = 0;
for i = 1 : size(odom_pose_msg, 1)
    odom_index = odom_pose_msg(i, 1);
    odom_index = odom_index{1, 1};
    
    if i == 1
        odom_t_start = odom_index.Header.Stamp.Sec + odom_index.Header.Stamp.Nsec / 1.e9;
    end
    odom_t(i, :) = odom_index.Header.Stamp.Sec + odom_index.Header.Stamp.Nsec / 1.e9;
    
    odom_x(i, :) = odom_index.Pose.Pose.Position.X;
    odom_y(i, :) = odom_index.Pose.Pose.Position.Y;

    %[odom_yaw(i, :), odom_pitch(i, :), odom_roll(i, :)] = quat2angle([odom_index.Pose.Pose.Orientation.X, odom_index.Pose.Pose.Orientation.Y, odom_index.Pose.Pose.Orientation.Z, odom_index.Pose.Pose.Orientation.W]);
    [odom_roll(i, :), odom_pitch(i, :), odom_yaw(i, :)] = quat2angle([odom_index.Pose.Pose.Orientation.X, odom_index.Pose.Pose.Orientation.Y, odom_index.Pose.Pose.Orientation.Z, odom_index.Pose.Pose.Orientation.W]);

    odom_vx(i, :) = odom_index.Twist.Twist.Linear.X;
    odom_wz(i, :) = odom_index.Twist.Twist.Angular.Z;

end

%%
ekf_pose_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/robot_position');
ekf_pose_msg = readMessages(ekf_pose_select);

ekf_pose_t_start = 0;
for i = 1 : size(ekf_pose_msg, 1)
    ekf_pose_index = ekf_pose_msg(i, 1);
    ekf_pose_index = ekf_pose_index{1, 1};
    
    if i == 1
        ekf_pose_t_start = ekf_pose_index.Header.Stamp.Sec + ekf_pose_index.Header.Stamp.Nsec / 1.e9;
    end
    ekf_pose_t(i, :) = ekf_pose_index.Header.Stamp.Sec + ekf_pose_index.Header.Stamp.Nsec / 1.e9;
    
    ekf_x(i, :) = ekf_pose_index.Pose.Position.X;
    ekf_y(i, :) = ekf_pose_index.Pose.Position.Y;

    [ekf_roll(i, :), ekf_pitch(i, :), ekf_yaw(i, :)] = quat2angle([ekf_pose_index.Pose.Orientation.X, ekf_pose_index.Pose.Orientation.Y, ekf_pose_index.Pose.Orientation.Z, ekf_pose_index.Pose.Orientation.W]);

end

%%
cmd_vel_select = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/cmd_vel');
cmd_vel_msg = readMessages(cmd_vel_select);

%cmd_vel_t_start = 0;
for i = 1 : size(cmd_vel_msg, 1)
    cmd_vel_index = cmd_vel_msg(i, 1);
    cmd_vel_index = cmd_vel_index{1, 1};
    
    %if i == 1
    %    cmd_vel_t_start = cmd_vel_index.Header.Stamp.Sec + cmd_vel_index.Header.Stamp.Nsec / 1.e9;
    %end
    %cmd_vel_t(i, :) = cmd_vel_index.Header.Stamp.Sec + cmd_vel_index.Header.Stamp.Nsec / 1.e9;
    
    cmd_vx(i, :) = cmd_vel_index.Linear.X;
    cmd_wz(i, :) = cmd_vel_index.Angular.Z;

end

% Plot Data
%%
% Fig.1 Map & Trajectories
%
close all
figure

x_start = min(ekf_x) - 0.5;
x_end   = max(ekf_x) + 0.5;
y_start = min(ekf_y) - 0.5;
y_end   = max(ekf_y) + 0.5;

for i = 5 : map_width - 5
    for j = 5 : map_height - 5
        x = double(i) * double(map_resolution) + map_x_offset;
        y = double(j) * double(map_resolution) + map_y_offset;
        map_val = map_2D(j, i);
        if map_val ~= -1
            rectangle('Position', [y, x, double(map_resolution), double(map_resolution)],'FaceColor', [255 - map_val 255 - map_val 255 - map_val] / 255, 'EdgeColor', [1. 1. 1.]);
            hold on
        end
    end
end


plot(ekf_x,   ekf_y)
hold on
plot(odom_x,  odom_y)
hold on
plot(laser_x, laser_y)
hold on

x_start = 0;
y_start = 0;
x_end   = 2.5;
y_end   = -1.05;
rectangle('Position', [x_start-0.1,y_start-0.1, 0.1,0.1],'FaceColor', [255 0 0] / 255, 'EdgeColor', [255 0 0] / 255, 'Curvature', [1, 1]);
rectangle('Position', [x_end-0.1,y_end-0.1, 0.1,0.1],    'FaceColor', [0 255 0] / 255, 'EdgeColor', [0 255 0] / 255, 'Curvature', [1, 1]);
axis equal

% Fig.2 X
figure
t_min = min([ekf_pose_t_start, odom_t_start, laser_t_start]);
plot(ekf_pose_t - t_min, ekf_x)
hold on
plot(odom_t - t_min,     odom_x)
hold on
plot(laser_t - t_min,    laser_x)

% Fig.3 Y
figure
%t_min = min([ekf_pose_t_start, odom_t_start, laser_t_start]);
plot(ekf_pose_t - t_min, ekf_y)
hold on
plot(odom_t - t_min,     odom_y)
hold on
plot(laser_t - t_min,    laser_y)

% Fig.4 Yaw
figure
%t_min = min([ekf_pose_t_start, odom_t_start, laser_t_start]);
plot(ekf_pose_t - t_min, ekf_yaw)
hold on
plot(odom_t - t_min,     odom_yaw)
hold on
plot(laser_t - t_min,    laser_yaw)

% Fig.5 Vx
% Fig.6 Wz
figure
plot(odom_t - t_min, odom_vx)
hold on
plot(odom_t - t_min, odom_wz)

% figure
% plot(cmd_vx)
% hold on
% plot(cmd_wz)


%imshow(map_2D);
